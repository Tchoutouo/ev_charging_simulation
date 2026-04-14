/**
 
 *  ALGORITHMES UTILISÉS :
 *   1. as_edge_graph()      → Graphe topologique depuis shapefile
 *   2. do goto / Dijkstra   → Plus court chemin sur graphe routier
 *   3. WSM pondéré          → Sélection de station multi-critères
 *   4. Ratio linéaire α/β   → Adaptation urgence batterie (fuzzy-like)
 *   5. FSM déterministe     → Contrôle des états véhicule
 *   6. File FIFO            → Gestion attente à la station
 *   7. Grille + proj. route → Placement spatial des stations
 *   8. Modèle kWh/km        → Consommation d'énergie réaliste
 *   9. Courbe CC-CV         → Recharge Li-ion réaliste
 *  10. SoH linéaire         → Dégradation batterie selon km parcourus
 *  11. Grid Search (batch)  → Exploration exhaustive des paramètres
 * ============================================================
 */

model EVChargingGIS

global {

    // -------------------------------------------------------
    // 1. FICHIERS GIS
    // -------------------------------------------------------
    file       roads_file <- file("../includes/roads.shp");
    geometry   shape      <- envelope(roads_file);

    // -------------------------------------------------------
    // 2. PARAMÈTRE TEMPOREL
    //    step = 5 s/cycle → mouvement   = 11.11 m/s × 5s = 55 m/cycle
    //    step = 60s/cycle → mouvement  = 11.11 m/s × 60s = 667 m/cycle  ← SAUT VISIBLE
    //    Un pas de 5 s donne un déplacement fluide et naturel sur la carte.
    // -------------------------------------------------------
    float step <- 5.0;   // 5 secondes / cycle = mouvement fluide

    // -------------------------------------------------------
    // 3. PARAMÈTRES DE SIMULATION (présentation)
    // -------------------------------------------------------
    int   nb_vehicles      <- 80;
    int   nb_stations      <- 5;   // 12 × 3 = 36 slots < 80 → file FIFO garantie
    int   station_capacity <- 3;
    
    // -------------------------------------------------------
    // NOUVEAUX AGENTS : TECHNICIENS
    // -------------------------------------------------------
    int   nb_technicians   <- 3;

    float initial_battery_min <- 10.0;   // Véhicules en crise dès t=0
    float initial_battery_max <- 100.0;

    // -------------------------------------------------------
    // 4. PARAMÈTRES INFRASTRUCTURE (Énergie & Économie)
    // -------------------------------------------------------
    float global_station_grid_power <- 300.0; // kW totaux partagés par la station
    float global_base_price         <- 0.30;  // Prix au kWh de base
    float nominal_charging_power_kw <- 150.0; // Prise individuelle max

    // -------------------------------------------------------
    // 5. SEUIL D'ARRIVÉE À LA STATION
    // -------------------------------------------------------
    float arrival_threshold <- 60.0;   // mètres

    // -------------------------------------------------------
    // 6. PARAMÈTRES SoH
    // -------------------------------------------------------
    float soh_degradation_per_1000km <- 2.0;
    float soh_min                    <- 70.0;

    // -------------------------------------------------------
    // 7. RÉCUPÉRATION DE PANNE
    //    Avec step=5s : 20 cycles = 100 s simulés (rapide pour la présentation)
    // -------------------------------------------------------
    int breakdown_recovery_cycles <- 80;   // 80 cycles × 5s = 400s simulées (visible)

    // -------------------------------------------------------
    // 7. DÉTECTION DE BLOCAGE (STUCK VEHICLE)
    //    Si un véhicule ne se déplace pas pendant max_stuck_cycles
    //    cycles consécutifs, sa cible est considérée inaccessible
    //    (tronçon isolé du graphe) et il choisit une nouvelle cible.
    // -------------------------------------------------------
    int max_stuck_cycles <- 3;   // Cycles sans mouvement avant de changer de cible

    // -------------------------------------------------------
    // 8. PARAMÈTRES STRATÉGIE WSM (ALGORITHMES 3 & 4)
    // -------------------------------------------------------
    float alpha_base <- 0.7;
    float beta_base  <- 0.3;

    // -------------------------------------------------------
    // 9. AFFICHAGE & CACHE ICÔNES
    //    Les image_file sont déclarées comme variables globales :
    //    GAMA les charge UNE SEULE FOIS à l'initialisation du modèle
    //    et les conserve en mémoire (cache automatique).
    //    Résultat : 0 accès disque par cycle d'affichage.
    // -------------------------------------------------------
    bool       use_icons         <- true;  // true = icônes PNG pré-chargées

    // -- Cache icônes véhicules --
    image_file icon_car_blue     <- image_file("../images/car_blue.png");
    image_file icon_car_orange   <- image_file("../images/car_orange.png");
    image_file icon_car_green    <- image_file("../images/car_green.png");
    image_file icon_car_red      <- image_file("../images/car_red.png");

    // -- Cache icônes stations --
    image_file icon_station_free <- image_file("../images/station_free.png");
    image_file icon_station_busy <- image_file("../images/station_busy.png");

    // -------------------------------------------------------
    // 10. VARIABLES GLOBALES DE MONITORING ET MÉTÉO
    // -------------------------------------------------------
    // --- MÉTÉO DYNAMIQUE ---
    float  rain_intensity <- 0.0;
    string weather_state  <- "sunny"; // "sunny" ou "rainy"
    int    weather_timer  <- 0;
    int    next_weather_change <- 400; // Cycles restants avant bascule (évalué à l'init)

    reflex update_weather {
        weather_timer <- weather_timer + 1;
        // Changer d'état 
        if (weather_timer > next_weather_change) {
            weather_state <- (weather_state = "sunny") ? "rainy" : "sunny";
            weather_timer <- 0;
            next_weather_change <- rnd(400, 600); // Prochain changement déterministe
        }

        if (weather_state = "rainy") {
            // La pluie monte progressivement (max 1.0)
            rain_intensity <- min(1.0, rain_intensity + 0.01);
        } else {
            // Le soleil revient progressivement
            rain_intensity <- max(0.0, rain_intensity - 0.01);
        }
    }

    graph        road_network;

    // Points naviguables garantis = endpoints des tronçons (nœuds du graphe).
    // Utiliser ces points élimine le risque de placer un véhicule sur un
    // segment isolé du graphe (cause racine de l'immobilité persistante).
    list<point>  navigable_locs <- [];

    // --- Compteurs événements ---
    int   total_breakdowns    <- 0;    // Nb total de pannes
    int   total_charges       <- 0;    // Nb total de recharges complètes
    float total_distance_km   <- 0.0;  // Distance fleet totale (km)
    float total_wait_time     <- 0.0;  // Attente cumulée des véhicules en file (cycles)
    int   total_recoveries    <- 0;    // Nb de retours après panne
    float total_energy_kwh    <- 0.0;  // Énergie totale rechargée (kWh)
    float total_consumed_kwh  <- 0.0;  // Énergie totale consommée (kWh)
    int   total_queue_entries <- 0;    // Nb d'entrées en file FIFO (pour calculer moyenne)

    // --- KPIs calculés chaque cycle ---
    float kpi_fleet_availability  <- 0.0;  // % véhicules opérationnels (pas broken)
    float kpi_charge_rate         <- 0.0;  // % véhicules en charge / flotte totale
    float kpi_avg_wait_per_charge <- 0.0;  // Attente moyenne par recharge (cycles)
    float kpi_breakdown_rate      <- 0.0;  // Pannes / 100 km (fiabilité)
    float kpi_avg_station_occ     <- 0.0;  // Taux occupation moyen des stations (%)
    float kpi_energy_efficiency   <- 0.0;  // kWh rechargés / kWh consommés
    float kpi_throughput          <- 0.0;  // Recharges complètes / 100 cycles
    float kpi_min_battery         <- 0.0;  // Batterie minimum dans la flotte (%)
    float kpi_stuck_rate          <- 0.0;  // % véhicules bloqués (stuck_counter > 0)
    int   kpi_total_queue_length  <- 0;    // Longueur totale des files d'attente

    // -------------------------------------------------------
    // 11. INITIALISATION
    // -------------------------------------------------------
    init {
        write "=== Chargement du réseau GIS ===";
        create road from: roads_file;

        // ALGORITHME 1 : as_edge_graph
        // Construit un graphe topologique pondéré depuis les segments shapefile.
        road_network <- as_edge_graph(road);
        write "Graphe construit : " + string(length(road)) + " tronçons.";

        // --- CALCUL DES POINTS NAVIGABLES (Graphe connexe) ---
        // On extrait uniquement les nœuds de la plus grande composante
        // connexe du graphe. Cela élimine définitivement les bouts de route
        // physiquement isolés du réseau principal (où véhicules/stations
        // seraient coincés et n'auraient aucun chemin reliant le reste de la ville).
        list<list> components <- connected_components_of(road_network);
        if (!empty(components)) {
            navigable_locs <- list<point>(components with_max_of (length(each)));
        }
        if (empty(navigable_locs)) {
            navigable_locs <- road collect each.shape.centroid;
        }
        write "Points navigables (nœuds CC principale) : " + length(navigable_locs);

        // ALGORITHME 7 : Placement des stations par grille régulière
        // Chaque station est projetée sur le nœud du graphe le plus proche
        // du centre de sa cellule de grille.
        write "=== Création des stations (algorithme de grille) ===";
        create charging_station number: nb_stations {
            capacity <- station_capacity;
        }

        // Copie locale pour éviter que plusieurs stations partagent le même nœud
        list<point> avail_locs <- copy(navigable_locs);
        int   grid_side <- int(sqrt(float(nb_stations))) + 1;
        float dw        <- world.shape.width  / float(grid_side);
        float dh        <- world.shape.height / float(grid_side);
        float ox        <- world.location.x - world.shape.width  / 2.0;
        float oy        <- world.location.y - world.shape.height / 2.0;

        int idx <- 0;
        loop s over: list(charging_station) {
            if (idx < nb_stations) {
                int   row         <- int(idx / grid_side);
                int   col         <- idx mod grid_side;
                point grid_center <- {ox + (col + 0.5) * dw, oy + (row + 0.5) * dh};
                // Utiliser un nœud réel du graphe (jamais un segment isolé)
                point closest <- avail_locs with_min_of (each distance_to grid_center);
                if (closest != nil) {
                    s.location <- closest;
                    avail_locs <- avail_locs - [closest];   // Un nœud par station max
                    if (empty(avail_locs)) { avail_locs <- copy(navigable_locs); }
                } else {
                    s.location <- one_of(navigable_locs);
                }
            }
            idx <- idx + 1;
        }
        write string(nb_stations) + " stations créées (sur nœuds du graphe).";

        write "=== Création des techniciens ===";
        create technician number: nb_technicians {
            location <- one_of(navigable_locs);
        }

        write "=== Création des véhicules ===";
        create vehicle number: nb_vehicles {
            // Placement sur un nœud réel du graphe = garanti navigable dès t=0
            location      <- one_of(navigable_locs);
            battery_level <- rnd(initial_battery_min, initial_battery_max);
            
            // --- HÉTÉROGÉNÉITÉ : Assignation des profils ---
            float proba <- rnd(1.0);
            if (proba < 0.60) {
                profile <- "taxi";
                battery_capacity_kwh   <- 20.0;
                base_energy_consumption_kwh <- 0.40;
                battery_threshold      <- 15.0;
                base_speed             <- 40.0 / 3.6;
                price_sensitivity      <- 0.2; // Préfère le temps (+ alpha) à l'argent
            } else if (proba < 0.80) {
                profile <- "delivery";
                battery_capacity_kwh   <- 60.0;
                base_energy_consumption_kwh <- 0.80;
                battery_threshold      <- 30.0;
                base_speed             <- 30.0 / 3.6; // Plus lent
                price_sensitivity      <- 0.5; // Équilibré
            } else {
                profile <- "personal";
                battery_capacity_kwh   <- 35.0;
                base_energy_consumption_kwh <- 0.30;
                battery_threshold      <- 40.0; // Anxieux de la panne
                base_speed             <- 40.0 / 3.6;
                price_sensitivity      <- 0.8; // Préfère détours pour bornes moins chères
            }
            
            speed <- base_speed;
            energy_consumption_kwh <- base_energy_consumption_kwh;
            
            state         <- "driving";
            initial_soh   <- rnd(85.0, 100.0);
            battery_soh   <- initial_soh;
        }
        write string(nb_vehicles) + " véhicules créés avec profils hétérogènes.";
        write "=== Simulation démarrée ===";
    }

    // -------------------------------------------------------
    // 12. COLLECTE DES INDICATEURS — calcul des KPIs chaque cycle
    //     Tous les indicateurs dérivés sont calculés ici centralement
    //     pour être disponibles dans les charts et moniteurs.
    // -------------------------------------------------------
    reflex collect_indicators {
        int   n_vehicles      <- length(vehicle);
        int   n_stations      <- length(charging_station);
        int   n_broken        <- vehicle count (each.state = "broken");
        int   n_charging      <- vehicle count (each.state = "charging");
        int   n_stuck         <- vehicle count (each.stuck_counter > 0);

        // KPI 1 — Disponibilité de la flotte (%)
        kpi_fleet_availability <- (n_vehicles > 0)
            ? (float(n_vehicles - n_broken) / float(n_vehicles)) * 100.0
            : 0.0;

        // KPI 2 — Taux de recharge instantané (%)
        kpi_charge_rate <- (n_vehicles > 0)
            ? (float(n_charging) / float(n_vehicles)) * 100.0
            : 0.0;

        // KPI 3 — Attente moyenne par recharge complète (cycles)
        kpi_avg_wait_per_charge <- (total_charges > 0)
            ? total_wait_time / float(total_charges)
            : 0.0;

        // KPI 4 — Taux de panne pour 100 km parcourus
        kpi_breakdown_rate <- (total_distance_km > 0.0)
            ? (float(total_breakdowns) / total_distance_km) * 100.0
            : 0.0;

        // KPI 5 — Occupation moyenne des stations (%)
        kpi_avg_station_occ <- (n_stations > 0)
            ? mean(charging_station collect (each.occupancy_rate())) * 100.0
            : 0.0;

        // KPI 6 — Efficacité énergétique (kWh rechargés / kWh consommés)
        kpi_energy_efficiency <- (total_consumed_kwh > 0.0)
            ? total_energy_kwh / total_consumed_kwh
            : 0.0;

        // KPI 7 — Débit de recharge (recharges complètes / 100 cycles)
        kpi_throughput <- (cycle > 0)
            ? (float(total_charges) / float(cycle)) * 100.0
            : 0.0;

        // KPI 8 — Batterie minimale dans la flotte (%)
        kpi_min_battery <- (n_vehicles > 0)
            ? min(vehicle collect (each.battery_level))
            : 0.0;

        // KPI 9 — Taux de véhicules bloqués (%)
        kpi_stuck_rate <- (n_vehicles > 0)
            ? (float(n_stuck) / float(n_vehicles)) * 100.0
            : 0.0;

        // KPI 10 — Longueur totale des files d'attente
        kpi_total_queue_length <- sum(charging_station collect (length(each.queue)));
    }
}

// =============================================================
//  ESPÈCE : road
// =============================================================
species road {
    float speed_limit <- 50.0;
    aspect base {
        draw shape color: rgb(100, 100, 100) width: 1;
    }
}

// =============================================================
//  ESPÈCE : charging_station
// =============================================================
species charging_station {

    int           capacity;
    int           occupied_slots <- 0;
    list<vehicle> queue          <- [];
    int           nb_served      <- 0;

    // ÉNERGIE ET ÉCONOMIE
    float current_price <- global_base_price;
    
    // Nouveaux ajouts : Résilience et Pannes
    bool isOperational <- true;
    bool is_getting_repaired <- false;
    
    reflex station_breakdown when: isOperational and flip(0.0005) {
        isOperational <- false;
    }

    // Mise à jour économique : le prix fluctue avec l'affluence (file d'attente incluse)
    reflex update_economics {
        float stress_factor <- (capacity > 0) ? (occupied_slots + length(queue)) / float(max(1, capacity * 2)) : 0.0;
        current_price <- global_base_price * (1.0 + stress_factor * 2.0); // Peut atteindre ~3x le prix initial
    }
    
    // Partage équitable de l'énergie du réseau électrique local
    float get_shared_power_kw {
        if (occupied_slots = 0) { return nominal_charging_power_kw; }
        return min(nominal_charging_power_kw, global_station_grid_power / float(occupied_slots));
    }

    // ALGORITHME 6 : File d'attente Intelligente et Réservation anticipée
    // Les véhicules sont triés par priorité.
    action add_to_queue (vehicle v) {
        if (!(v in queue)) {
            add v to: queue;
            // Tri par priorité décroissante (3 = haut, 1 = bas)
            queue <- reverse(queue sort_by (each.get_priority()));
            total_queue_entries <- total_queue_entries + 1;  // Compteur d'entrées en file
        }
    }

    action start_charging {
        loop while: (occupied_slots < capacity) and (not empty(queue)) and isOperational {
            // Prendre le véhicule le plus prioritaire qui est DEJA arrivé à la station
            vehicle nxt <- queue first_with (each.reached_station = true);
            if (nxt != nil) {
                remove nxt from: queue;
                occupied_slots <- occupied_slots + 1;
                ask nxt {
                    state              <- "charging";
                    total_wait_time    <- total_wait_time + float(cycle - waiting_time_start);
                }
            } else {
                break; // Aucun véhicule dans la file n'est physiquement arrivé
            }
        }
    }

    action release_vehicle (vehicle v) {
        occupied_slots <- max(0, occupied_slots - 1);
        nb_served      <- nb_served + 1;
        total_charges  <- total_charges + 1;
        do start_charging;
    }

    bool  is_full        { return occupied_slots >= capacity or !isOperational; }
    float occupancy_rate { return (capacity = 0) ? 0.0 : float(occupied_slots) / float(capacity); }

    // ASPECT PRINCIPALE : utilise les icônes depuis le cache global
    aspect icon_aspect {
        float icon_size <- 40.0;
        image_file cur_img <- icon_station_busy;
        
        if (!isOperational) {
            // Station HS : filtre gris / indication
            draw icon_station_busy size: {icon_size, icon_size} color: rgb(100,100,100);
            draw "HS" at: {location.x, location.y} color: #red font: font("Arial", 12, #bold);
            return;
        }
        
        cur_img <- (occupancy_rate() < 0.8) ? icon_station_free : icon_station_busy;
        draw cur_img size: {icon_size, icon_size};
        draw string(occupied_slots) + "/" + string(capacity)
             at: {location.x, location.y + icon_size * 0.7}
             color: #white font: font("Arial", 9, #bold);
        if (not empty(queue)) {
            draw "Q:" + string(length(queue))
                 at: {location.x + icon_size * 0.6, location.y}
                 color: #yellow font: font("Arial", 8, #bold);
        }
    }

    aspect base {
        int rv <- int(255.0 * occupancy_rate());
        int gv <- int(255.0 * (1.0 - occupancy_rate()));
        draw circle(15) color: rgb(rv, gv, 0) border: #black;
        draw string(occupied_slots) + "/" + string(capacity)
             at: {location.x, location.y + 20}
             color: #black font: font("Arial", 9, #bold);
        if (not empty(queue)) {
            draw "Q:" + string(length(queue))
                 at: {location.x + 18, location.y}
                 color: #darkorange font: font("Arial", 8, #plain);
        }
    }
}


// =============================================================
//  ESPÈCE : vehicle  (Skill Moving — déplacement GIS)
//
//  ALGORITHME 5 : Automate à états finis déterministe (FSM/DFA)
//  ┌──────────┐  batterie < seuil  ┌───────────┐  station trouvée  ┌─────────┐
//  │ driving  ├───────────────────►│ searching ├──────────────────►│ queuing │
//  └──────────┘                    └───────────┘                    └────┬────┘
//       ▲                                                                │ arrivé + place libre
//       │ chargé ≥ SoH×0.95                               ┌─────────────▼──────┐
//       └──────────────────────────────────────────────────┤    charging        │
//                                                          └────────────────────┘
//  broken : état transitoire → après N cycles → searching (dépannage)
// =============================================================
species vehicle skills: [moving] {

    float            battery_level;
    string           state;
    charging_station target_station    <- nil;
    point            target_point      <- nil;
    float            distance_traveled <- 0.0;   // mètres
    int              waiting_time_start <- 0;

    // --- HÉTÉROGÉNÉITÉ & ÉCONOMIE ---
    string           profile;
    float            battery_capacity_kwh;
    float            base_energy_consumption_kwh;
    float            energy_consumption_kwh;
    float            battery_threshold;
    float            price_sensitivity;
    float            base_speed;
    int              parked_counter <- 0;
    bool             is_getting_repaired <- false;
    
    // NOUVEAU : Adapter la vitesse et conso à la pluie
    reflex adapt_to_weather {
        speed <- base_speed * (1.0 - (0.4 * rain_intensity));
        energy_consumption_kwh <- base_energy_consumption_kwh * (1.0 + (0.2 * rain_intensity));
    }
    
    // Réservation et priorité
    float targetArrivalTime <- 0.0;
    
    int get_priority {
        if (profile = "delivery") { return 3; }
        else if (profile = "taxi") { return 2; }
        else { return 1; }
    }

    // ALGORITHME 10 — SoH (State of Health)
    float initial_soh <- 100.0;
    float battery_soh <- 100.0;
    float total_km    <- 0.0;

    // Récupération de panne
    int   breakdown_cycle <- 0;

    // [CORRECTION BUG 3] Flag : le véhicule a-t-il physiquement navigué
    // jusqu'à la station avant d'entrer en recharge ?
    bool reached_station <- false;

    // [CORRECTION BUG FREEZE] Compteur de cycles sans mouvement.
    // Si stuck_counter >= max_stuck_cycles → cible inaccessible → nouvelle cible.
    int stuck_counter <- 0;

    // [CORRECTION BUG ORANGE IMMOBILE]
    // Liste noire des stations inaccessibles pour ce véhicule.
    list<charging_station> blacklisted_stations <- [];

    // Compteur de téléportations
    int stuck_resets <- 0;

    // Compteur d'attente en file (timeout anti-blocage)
    int queue_wait_counter <- 0;

    // -------------------------------------------------------
    // FSM — ÉTAT : driving
    //   Déplacement aléatoire tant que batterie > seuil.
    //   Passe à "searching" dès que batterie < battery_threshold.
    // -------------------------------------------------------
    reflex do_drive when: (state = "driving") {
        if (battery_level <= 0.0) { do breakdown; return; }

        // NOUVEAU: Anticipation = seuil critique OU batterie insuffisante pour faire le trajet + marge station
        bool need_charge <- false;
        if (battery_level < battery_threshold) {
            need_charge <- true;
        } else if (target_point != nil) {
            float dist_to_target <- location distance_to target_point;
            // On ajoute 2 km de sécurité pour trouver une station depuis la cible
            float required_energy_kwh <- energy_consumption_kwh * ((dist_to_target + 2000.0) / 1000.0);
            float required_soc <- (required_energy_kwh / battery_capacity_kwh) * 100.0;
            if (battery_level < required_soc) {
                need_charge <- true;
            }
        }

        if (need_charge) {
            state           <- "searching";
            target_point    <- nil;
            target_station  <- nil;
            reached_station <- false;
            // Pas de return : le véhicule continue de se déplacer ce cycle
        }

        // --- HÉTÉROGÉNÉITÉ : Transition vers l'état 'parked' ---
        if (state = "driving" and profile = "personal" and target_point = nil and flip(0.005)) {
            // Un particulier qui se balade sans but précis a une probabilité
            // de trouver sa destination et de stationner (ex: 5 à 15 minutes simulées).
            state          <- "parked";
            target_point   <- nil;
            parked_counter <- rnd(50, 150); 
            return;
        }

        // Cible aléatoire sur un nœud réel du graphe (jamais un segment isolé)
        if (target_point = nil) {
            target_point  <- one_of(navigable_locs);
            stuck_counter <- 0;
        }

        // ALGORITHME 2 : Dijkstra via do goto
        point prev_loc  <- location;
        do goto target: target_point on: road_network speed: speed;
        float dist_moved <- location distance_to prev_loc;
        do consume_energy_for(dist_moved);

        // --- DÉTECTION DE BLOCAGE ---
        if (dist_moved < 1.0 and location distance_to target_point > arrival_threshold) {
            stuck_counter <- stuck_counter + 1;
            if (stuck_counter >= max_stuck_cycles) {
                stuck_resets  <- stuck_resets + 1;
                target_point  <- one_of(navigable_locs);   // Nouvelle cible sur le graphe
                stuck_counter <- 0;
                // Téléportation d'urgence après 3 blocages consécutifs
                // (le véhicule était sur un segment physiquement isolé)
                if (stuck_resets >= 3) {
                    location     <- one_of(navigable_locs);
                    stuck_resets <- 0;
                    write "[TÉLÉPORT] Véhicule " + int(self) + " repositionné sur le graphe.";
                }
            }
        } else {
            stuck_counter <- 0;
            stuck_resets  <- 0;   // Reset si le véhicule se déplace normalement
            if (location distance_to target_point < arrival_threshold) {
                target_point <- nil;
            }
        }
    }

    // -------------------------------------------------------
    // FSM — ÉTAT : parked (Exclusif aux particuliers)
    // -------------------------------------------------------
    reflex do_parked when: (state = "parked") {
        parked_counter <- parked_counter - 1;
        
        // S'il est garé mais s'aperçoit que sa batterie est critique, il sort pour charger
        if (battery_level < battery_threshold) {
             state <- "searching";
             return;
        }
        
        if (parked_counter <= 0) {
            state <- "driving";
        }
    }

    // -------------------------------------------------------
    // FSM — ÉTAT : searching
    //   SéPARATION NETTE : do_search sélectionne une station ET retourne.
    //   Si aucune station n'est trouvée, le véhicule se déplace aléatoirement.
    //   BUG CORRIGÉ : return après transition empêche la détection de
    //   blocage de blacklister la station dans le même cycle.
    // -------------------------------------------------------
    reflex do_search when: (state = "searching") {
        if (battery_level <= 0.0) { do breakdown; return; }

        // --- INLINE SELECTION DE STATION (Remplace l'action pour éviter les bugs GAMA) ---
        list<charging_station> candidates <- list(charging_station) - blacklisted_stations;
        if (empty(candidates)) {
            blacklisted_stations <- [];
            candidates <- list(charging_station);
        }
        
        charging_station best <- nil;
        
        if (!empty(candidates)) {
            list<charging_station> free_stations <- candidates where (!each.is_full());
            if (!empty(free_stations)) {
                best <- free_stations with_min_of (self distance_to each.location);
            } else {
                float ratio <- (battery_threshold > 0.0) ? (battery_level / battery_threshold) : 1.0;
                float gamma        <- price_sensitivity;
                float non_fin_w    <- 1.0 - gamma;
                float alpha_factor <- non_fin_w * min(1.0, ratio);
                float alpha        <- alpha_factor; 
                float beta         <- non_fin_w - alpha_factor;

                float max_dist  <- max(candidates collect (self distance_to each.location));
                float max_queue <- float(max(candidates collect length(each.queue)));
                float max_price <- max(candidates collect each.current_price);
                if (max_dist  <= 0.0) { max_dist  <- 1.0; }
                if (max_queue <= 0.0) { max_queue <- 1.0; }
                if (max_price <= 0.0) { max_price <- 1.0; }

                float best_score <- #infinity;
                loop s over: candidates {
                    float nd <- (self distance_to s.location) / max_dist;
                    float nq <- float(length(s.queue)) / max_queue;
                    float np <- s.current_price / max_price;
                    float sc <- alpha * nd + beta * nq + gamma * np;
                    if (sc < best_score) { best_score <- sc; best <- s; }
                }
            }
        }
        // --- FIN INLINE ---

        if (best != nil) {
            // Station trouvée → transition propre vers queuing
            target_station     <- best;
            target_point       <- nil;
            reached_station    <- false;
            stuck_counter      <- 0;
            stuck_resets       <- 0;
            queue_wait_counter <- 0;
            state              <- "queuing";
            
            // Réservation anticipée : on intègre la file d'attente immédiatement !
            ask target_station { do add_to_queue(myself); }
            waiting_time_start <- cycle;
            
            return;   // do_queue prend le relais au prochain cycle
        }

        // Aucune station accessible → déplacement aléatoire
        if (target_point = nil) {
            target_point  <- one_of(navigable_locs);
            stuck_counter <- 0;
        }
        point prev_loc <- location;
        do goto target: target_point on: road_network speed: speed;
        float dist_moved <- location distance_to prev_loc;
        do consume_energy_for(dist_moved);

        if (dist_moved < 1.0 and location distance_to target_point > arrival_threshold) {
            stuck_counter <- stuck_counter + 1;
            if (stuck_counter >= max_stuck_cycles) {
                stuck_resets  <- stuck_resets + 1;
                target_point  <- one_of(navigable_locs);
                stuck_counter <- 0;
                if (stuck_resets >= 3) {
                    location     <- one_of(navigable_locs);
                    stuck_resets <- 0;
                }
            }
        } else {
            stuck_counter <- 0;
            stuck_resets  <- 0;
            if (location distance_to target_point < arrival_threshold) {
                target_point <- nil;
            }
        }
    }

    // -------------------------------------------------------
    // FSM — ÉTAT : queuing
    //   Le véhicule navigue vers la station cible.
    //   La détection d'arrivée utilise arrival_threshold (100 m).
    // -------------------------------------------------------
    reflex do_queue when: (state = "queuing") {
        if (battery_level <= 0.0) {
            if (target_station != nil and (self in target_station.queue)) {
                ask target_station { remove myself from: queue; }
            }
            do breakdown;
            return;
        }

        if (target_station = nil or !target_station.isOperational) {
            if (target_station != nil) {
                ask target_station { remove myself from: queue; }
                if (!(target_station in blacklisted_stations)) {
                    add target_station to: blacklisted_stations;
                }
            }
            reached_station    <- false;
            queue_wait_counter <- 0;
            state              <- "searching";
            return;
        }

        float dist_to_station <- location distance_to target_station.location;

        if (!reached_station and dist_to_station > arrival_threshold) {
            // Se déplacer vers la station
            point prev_loc   <- location;
            do goto target: target_station.location on: road_network speed: speed;
            float dist_moved <- location distance_to prev_loc;
            do consume_energy_for(dist_moved);

            if (dist_moved < 1.0 and dist_to_station > arrival_threshold) {
                stuck_counter <- stuck_counter + 1;
                if (stuck_counter >= max_stuck_cycles) {
                    if (!(target_station in blacklisted_stations)) {
                        add target_station to: blacklisted_stations;
                    }
                    target_station     <- nil;
                    reached_station    <- false;
                    stuck_counter      <- 0;
                    queue_wait_counter <- 0;
                    state              <- "searching";
                }
            } else {
                stuck_counter <- 0;
            }

        } else {
            // Arrivée à la station confirmée
            reached_station <- true;
            stuck_counter   <- 0;

            if (self in target_station.queue) {
                // Véhicule en file d'attente
                queue_wait_counter <- queue_wait_counter + 1;

                // [NOUVEAU] On informe la station qu'on est là pour qu'elle puisse lancer la charge si slot libre
                ask target_station { do start_charging; }

                // Timeout : abandon après 120 cycles (=10 min simulées) en file
                if (queue_wait_counter > 120) {
                    ask target_station { remove myself from: queue; }
                    if (!(target_station in blacklisted_stations)) {
                        add target_station to: blacklisted_stations;
                    }
                    target_station     <- nil;
                    reached_station    <- false;
                    queue_wait_counter <- 0;
                    state              <- "searching";
                }

            } else {
                // Véhicule arrivé mais pas encore dans la file
                queue_wait_counter <- 0;
                if (!target_station.is_full()) {
                    ask target_station { occupied_slots <- occupied_slots + 1; }
                    state <- "charging";
                } else {
                    ask target_station { do add_to_queue(myself); }
                    waiting_time_start <- cycle;
                }
            }
        }
    }

    // -------------------------------------------------------
    // FSM — ÉTAT : charging
    //   ALGORITHME 9 : Courbe de recharge CC-CV (Li-ion réaliste)
    //   0%→80%  : Phase CC (Courant Constant) — puissance nominale
    //   80%→100%: Phase CV (Tension Constante) — puissance décroissante
    // -------------------------------------------------------
    reflex do_charge when: (state = "charging") {

        // Garde : incohérence d'état possible ou borne tombée en panne pendant qu'on charge
        if (target_station = nil or !target_station.isOperational) {
            if (target_station != nil) {
                // Se détacher silencieusement de la station en libérant le port physique
                ask target_station { occupied_slots <- max(0, occupied_slots - 1); }
                if (!(target_station in blacklisted_stations)) {
                    add target_station to: blacklisted_stations;
                }
            }
            state <- "searching";
            target_station <- nil;
            reached_station <- false;
            return;
        }

        float hours_per_cycle    <- step / 3600.0;
        float effective_power_kw;
        float allocated_power <- target_station.get_shared_power_kw();

        // Modélisation physique avancée : limite selon type de véhicule
        float max_vehicle_power <- (profile = "delivery") ? 200.0 : ((profile = "taxi") ? 100.0 : 50.0);
        allocated_power <- min(allocated_power, max_vehicle_power);

        // Limite selon State of Health (SoH) : une batterie usée accepte moins de puissance pour éviter la surchauffe
        float soh_factor <- max(0.5, battery_soh / 100.0);
        allocated_power <- allocated_power * soh_factor;

        if (battery_level < 80.0) {
            // Phase CC : puissance complète allouée (limitée physiquement)
            effective_power_kw <- allocated_power;
        } else {
            // Phase CV : dégradation linéaire proportionnelle à l'allocation
            float soc_factor   <- 1.0 - ((battery_level - 80.0) / 20.0) * 0.9;
            effective_power_kw <- allocated_power * max(0.1, soc_factor);
        }

        float energy_charged_kwh <- effective_power_kw * hours_per_cycle;
        float delta_soc          <- (energy_charged_kwh / battery_capacity_kwh) * 100.0;
        float effective_max      <- battery_soh;   // Plafond SoH

        battery_level    <- min(effective_max, battery_level + delta_soc);
        total_energy_kwh <- total_energy_kwh + energy_charged_kwh;  // Énergie rechargée cumulée

        // Fin de charge : libérer la place et reprendre la route
        if (battery_level >= effective_max * 0.95) {
            ask target_station { do release_vehicle(myself); }
            target_station  <- nil;
            reached_station <- false;
            state           <- "driving";
        }
    }

    // -------------------------------------------------------
    // FSM — ÉTAT : broken
    //   Clignotement visuel via cycle mod.
    //   Le dépannage s'effectue par l'intervention externe d'un technicien (espèce technician).
    // -------------------------------------------------------


    // -------------------------------------------------------
    // ACTION : Consommation d'énergie sur distance réelle
    //   effectivement parcourue par do goto (pas la distance théorique).
    //   ALGORITHME 8 : ΔSoC = (conso_kWh/km × dist_km / capacité_kWh) × 100
    // -------------------------------------------------------
    action consume_energy_for (float actual_dist_m) {
        if (actual_dist_m <= 0.0) { return; }

        float dist_km    <- actual_dist_m / 1000.0;
        float energy_kwh <- energy_consumption_kwh * dist_km;
        float delta_soc  <- (energy_kwh / battery_capacity_kwh) * 100.0;

        battery_level     <- max(0.0, battery_level - delta_soc);
        distance_traveled <- distance_traveled + actual_dist_m;
        total_km          <- total_km + dist_km;
        total_distance_km <- total_distance_km + dist_km;
        total_consumed_kwh <- total_consumed_kwh + energy_kwh;  // Énergie consommée cumulée

        // ALGORITHME 10 : Mise à jour SoH
        do update_soh;
    }

    // -------------------------------------------------------
    // ACTION : Mise à jour du State of Health (SoH)
    //   ALGORITHME 10 : Dégradation linéaire
    //   SoH(km) = max(soh_min, initial_soh - (km/1000) × taux_dégradation)
    // -------------------------------------------------------
    action update_soh {
        float new_soh <- max(soh_min, initial_soh - (total_km / 1000.0) * soh_degradation_per_1000km);
        battery_soh   <- new_soh;
        if (battery_level > battery_soh) {
            battery_level <- battery_soh;
        }
    }

    // -------------------------------------------------------
    // ACTION : Passage en panne
    // -------------------------------------------------------
    action breakdown {
        if (target_station != nil) {
            if (self in target_station.queue) {
                // Véhicule en file → retirer de la file
                ask target_station { remove myself from: queue; }
            } else if (state = "charging") {
                ask target_station {
                    occupied_slots <- max(0, occupied_slots - 1);
                    do start_charging;   // Libérer les véhicules en attente
                }
            }
        }
        state                <- "broken";
        target_station       <- nil;
        target_point         <- nil;
        reached_station      <- false;
        stuck_counter        <- 0;
        queue_wait_counter   <- 0;
        breakdown_cycle      <- 0;
        blacklisted_stations <- [];
        total_breakdowns     <- total_breakdowns + 1;
    }

    // -------------------------------------------------------
    // VISUALISATION — Aspect principal avec clignotement panne
    //   Clignotement : calculé directement en aspect via cycle mod
    //   (plus fiable qu'un flag booléen mis à jour dans un reflex).
    // -------------------------------------------------------
    // -------------------------------------------------------
    // ASPECT PRINCIPAL : icônes depuis le cache global
    //   icon_size agrandi (28px) pour une meilleure visibilité des couleurs d'état.
    // -------------------------------------------------------
    aspect icon_aspect {
        float icon_size  <- 28.0;
        // Clignotement panne : 4 cycles visible / 4 cycles invisible
        // À step=5s et GAMA vitesse normale (~5-10 c/s réel) → ~0.5s de période
        bool  blink_show <- (state != "broken") or ((cycle mod 8) < 4);

        image_file cur_img <- nil;
        if      (state = "driving")   { cur_img <- icon_car_blue; }
        else if (state = "searching") { cur_img <- icon_car_orange; }
        else if (state = "queuing")   { cur_img <- icon_car_orange; }
        else if (state = "charging")  { cur_img <- icon_car_green; }
        else if (blink_show)          { cur_img <- icon_car_red; }

        if (cur_img != nil) {
            draw cur_img size: {icon_size, icon_size};
        }

        // Barre de batterie — proportionnelle à l'icône
        float bar_w  <- icon_size;
        float fill_w <- max(0.5, bar_w * (battery_level / 100.0));
        draw rectangle(bar_w, 4.0)
             at: {location.x, location.y - icon_size * 0.72}
             color: rgb(40, 40, 40) border: #black;
        int br <- int(255.0 * (1.0 - battery_level / 100.0));
        int bg <- int(255.0 * (battery_level / 100.0));
        draw rectangle(fill_w, 4.0)
             at: {location.x - (bar_w - fill_w) / 2.0, location.y - icon_size * 0.72}
             color: rgb(br, bg, 0);
             
        // Valeur Numérique SoC au-dessus
        draw string(int(battery_level)) + "%"
             at: {location.x - icon_size * 0.3, location.y - icon_size * 1.1}
             color: rgb(br, bg, 0) font: font("Arial", 10, #bold);

        // Label SoH dégradé (< 95%)
        if (battery_soh < 95.0) {
            draw "SoH:" + string(int(battery_soh)) + "%"
                 at: {location.x + icon_size * 0.55, location.y + icon_size * 0.3}
                 color: #yellow font: font("Arial", 6, #plain);
        }
        
        // Label Profil Hétérogène
        string lbl <- (profile="taxi") ? "TAXI" : ((profile="delivery") ? "LIVR" : "");
        if (lbl != "") {
             draw lbl at: {location.x - icon_size*0.4, location.y - icon_size*0.9}
                  color: (profile="taxi") ? #yellow : #cyan font: font("Arial", 5, #bold);
        }
    }

    // -------------------------------------------------------
    // ASPECT : base — Affichage en couleurs directes (sans images)
    //   Couleurs FSM :
    //     driving   → bleu
    //     searching → orange
    //     queuing   → orange foncé
    //     charging  → vert
    //     broken    → rouge clignotant
    // -------------------------------------------------------
    aspect base {
        bool blink_show <- (state != "broken") or ((cycle mod 10) < 5);
        rgb  col;
        int  al <- 255;

        if      (state = "driving")   { col <- rgb(30,  144, 255); }  // bleu
        else if (state = "searching") { col <- rgb(255, 165,   0); }  // orange
        else if (state = "queuing")   { col <- rgb(255,  80,   0); }  // orange vif
        else if (state = "charging")  { col <- rgb( 50, 220,  50); }  // vert
        else {
            col <- rgb(220, 20, 60);                                   // rouge
            al  <- blink_show ? 255 : 15;
        }

        // Cercle principal plus grand (10px) = état bien visible
        draw circle(10) color: rgb(col.red, col.green, col.blue, al) border: #black;

        // Barre de batterie (16 x 3 px)
        float bar_w  <- 16.0;
        float fill_w <- max(0.5, bar_w * (battery_level / 100.0));
        draw rectangle(bar_w, 3.0)
             at: {location.x, location.y - 13.0}
             color: rgb(40, 40, 40) border: rgb(20, 20, 20);
        int br <- int(255.0 * (1.0 - battery_level / 100.0));
        int bg <- int(255.0 * (battery_level / 100.0));
        draw rectangle(fill_w, 3.0)
             at: {location.x - (bar_w - fill_w) / 2.0, location.y - 13.0}
             color: rgb(br, bg, 0);
             
        // Valeur Numérique SoC au-dessus
        draw string(int(battery_level)) + "%"
             at: {location.x - 8.0, location.y - 22.0}
             color: rgb(br, bg, 0) font: font("Arial", 9, #bold);
    }
}

// =============================================================
//  ESPÈCE : technician
// =============================================================
species technician skills: [moving] {
    string state <- "patrolling";
    agent target_agent <- nil;
    int repair_timer <- 0;
    float speed <- 50.0 / 3.6;

    reflex adapt_to_weather {
        speed <- (50.0 / 3.6) * (1.0 - (0.4 * rain_intensity));
    }

    reflex search_incident when: state = "patrolling" {
        // Chercher une station en panne
        charging_station broken_st <- one_of(charging_station where (!each.isOperational and !each.is_getting_repaired));
        if (broken_st != nil) {
            target_agent <- broken_st;
            broken_st.is_getting_repaired <- true;
            state <- "dispatched";
            return;
        }

        // Chercher un véhicule en panne
        vehicle broken_v <- one_of(vehicle where (each.state = "broken" and !each.is_getting_repaired));
        if (broken_v != nil) {
            target_agent <- broken_v;
            broken_v.is_getting_repaired <- true;
            state <- "dispatched";
            return;
        }

        // Patrouille aléatoire
        do goto target: one_of(navigable_locs) on: road_network speed: speed;
    }

    reflex go_to_incident when: state = "dispatched" {
        if (target_agent = nil) { state <- "patrolling"; return; }
        
        do goto target: target_agent.location on: road_network speed: speed;
        
        if (location distance_to target_agent.location < 15.0) {
            state <- "repairing";
            repair_timer <- 20; // 20 cycles pour réparer (~100s)
        }
    }

    reflex repair when: state = "repairing" {
        repair_timer <- repair_timer - 1;
        if (repair_timer <= 0) {
            if (target_agent is charging_station) {
                charging_station st <- charging_station(target_agent);
                st.isOperational <- true;
                st.is_getting_repaired <- false;
            } else if (target_agent is vehicle) {
                vehicle v <- vehicle(target_agent);
                v.battery_level <- 25.0; // Recharge d'urgence via dépanneuse
                v.state <- "searching";
                v.reached_station <- false;
                v.target_station <- nil;
                v.target_point <- nil;
                v.stuck_counter <- 0;
                v.blacklisted_stations <- [];
                v.is_getting_repaired <- false;
                v.breakdown_cycle <- 0;
                total_recoveries <- total_recoveries + 1;
            }
            target_agent <- nil;
            state <- "patrolling";
        }
    }

    aspect base {
        draw triangle(15) color: #magenta border: #white;
        if (state = "repairing") {
            draw "FIXING" at: {location.x, location.y - 15} color: #white font: font("Arial", 8, #bold);
        } else if (state = "dispatched") {
            draw "SOS" at: {location.x, location.y - 15} color: #magenta font: font("Arial", 8, #bold);
        }
    }
}

// =============================================================
//  EXPÉRIMENTATION GUI
// =============================================================
experiment EVSimulation type: gui {

    parameter "Nb véhicules"               var: nb_vehicles              min: 1      max: 500    category: "Simulation";
    parameter "Nb stations"                var: nb_stations              min: 1      max: 100    category: "Simulation";
    parameter "Capacité station"           var: station_capacity         min: 1      max: 20     category: "Simulation";
    parameter "Pas de temps (s/cycle)"     var: step                     min: 5.0    max: 120.0  category: "Simulation";
    parameter "Seuil arrivée station (m)"  var: arrival_threshold        min: 20.0   max: 500.0  category: "Simulation";
    parameter "Cycles max sans mouvement"  var: max_stuck_cycles         min: 1      max: 20     category: "Simulation";

    parameter "Batterie initiale min (%)"  var: initial_battery_min      min: 10.0   max: 100.0  category: "Batterie";
    parameter "Batterie initiale max (%)"  var: initial_battery_max      min: 10.0   max: 100.0  category: "Batterie";
    parameter "Dégradation SoH (%/1000km)" var: soh_degradation_per_1000km min: 0.0  max: 2.0   category: "Batterie";

    // Paramètres liés aux infrastructures énergétiques
    parameter "Puissance Grille(kW)"       var: global_station_grid_power min: 100.0 max: 1000.0 category: "Infrastructures";
    parameter "Prix de base (€/kWh)"       var: global_base_price         min: 0.1   max: 1.0    category: "Infrastructures";

    parameter "Alpha (poids distance)"     var: alpha_base               min: 0.0    max: 1.0    category: "Stratégie";
    parameter "Beta (poids file)"          var: beta_base                min: 0.0    max: 1.0    category: "Stratégie";
    parameter "Délai récupération (cycles)" var: breakdown_recovery_cycles min: 10   max: 500    category: "Stratégie";

    parameter "Utiliser les icônes"        var: use_icons                                        category: "Affichage";

    output {

        display "Carte GIS" type: opengl background: rgb(20, 20, 30) {
            species road             aspect: base;
            species charging_station aspect: icon_aspect;
            species technician       aspect: base;
            species vehicle          aspect: icon_aspect;
            
            // Météo Overlay
            graphics "Weather" {
                if (rain_intensity > 0.0) {
                    draw "MÉTÉO: PLUIE (" + string(int(rain_intensity*100)) + "%)" at: {world.shape.width/2 - 200, 200} color: #cyan font: font("Arial", 20, #bold);
                } else {
                    draw "MÉTÉO: SOLEIL" at: {world.shape.width/2 - 200, 200} color: #yellow font: font("Arial", 20, #bold);
                }
            }
        }

        // TABLEAU DE BORD 1 — États & Flux
        display "États & Flux" refresh: every(1 #cycles) {

            chart "Distribution des états (véhicules)" type: pie background: #white {
                data "🚗 Roulant"   value: vehicle count (each.state = "driving")   color: #dodgerblue;
                data "🔍 Recherche" value: vehicle count (each.state = "searching") color: #orange;
                data "⏳ En file"   value: vehicle count (each.state = "queuing")   color: #darkorange;
                data "⚡ En charge" value: vehicle count (each.state = "charging")  color: #limegreen;
                data "💥 En panne"  value: vehicle count (each.state = "broken")    color: #red;
            }

            chart "Flux d'événements cumulés" type: series background: #white {
                data "Pannes"        value: total_breakdowns color: #red    style: line;
                data "Recharges"     value: total_charges    color: #green  style: line;
                data "Récupérations" value: total_recoveries color: #orange style: line;
            }

            chart "Disponibilité de la flotte (%)" type: series background: #white {
                data "Disponibilité" value: kpi_fleet_availability color: #limegreen style: line;
                data "Taux en charge" value: kpi_charge_rate       color: #steelblue style: line;
                data "Taux bloqués"   value: kpi_stuck_rate        color: #red       style: line;
            }

            chart "Taille des files d'attente" type: series background: #white {
                data "File totale" value: kpi_total_queue_length color: #purple style: line;
            }
        }

        // TABLEAU DE BORD 2 — Énergie & Batterie
        display "Énergie & Batterie" refresh: every(1 #cycles) {

            chart "Niveaux batterie (%)" type: series background: #white {
                data "Batterie moy." value: mean(vehicle collect (each.battery_level)) color: #green  style: line;
                data "SoH moyen"     value: mean(vehicle collect (each.battery_soh))   color: #blue   style: line;
                data "Batterie min." value: kpi_min_battery                            color: #orange style: line;
                data "Seuil al. moy." value: mean(vehicle collect (each.battery_threshold)) color: #red    style: line;
            }

            chart "Énergie totale (kWh)" type: series background: #white {
                data "Rechargée"   value: total_energy_kwh   color: #limegreen style: line;
                data "Consommée"   value: total_consumed_kwh color: #red       style: line;
            }

            chart "Efficacité énergétique" type: series background: #white {
                data "kWh rechargés / kWh consommés" value: kpi_energy_efficiency color: #cyan style: line;
            }

            // -------------------------------------------------------
            // GRAPHIQUE SoH — DYNAMIQUE (série temporelle)
            //   Remplace l'histogramme statique qui avait :
            //   - Labels tronqués/chevauchants
            //   - Toutes les barres à ~100% au début (illisible)
            //   Maintenant : évolution min/moy/max visible dès les premiers cycles.
            // -------------------------------------------------------
            chart "Evolution SoH de la flotte (%)" type: series background: #white {
                data "SoH max"      value: max(vehicle collect each.battery_soh)  color: #limegreen   style: line;
                data "SoH moyen"    value: mean(vehicle collect each.battery_soh) color: #dodgerblue  style: line;
                data "SoH min"      value: min(vehicle collect each.battery_soh)  color: #red         style: line;
                data "Seuil 80%"    value: 80.0                                   color: #orange      style: line;
                data "Seuil remp."  value: 70.0                                   color: #darkred     style: line;
            }

            // Snapshot distribution SoH instantanée (s'enrichit avec le temps)
            chart "Santé flotte — instantané" type: pie background: #white {
                data ">= 95%" value: vehicle count (each.battery_soh >= 95.0)                              color: #limegreen;
                data "90-95" value: vehicle count (each.battery_soh >= 90.0 and each.battery_soh < 95.0) color: #yellowgreen;
                data "85-90" value: vehicle count (each.battery_soh >= 85.0 and each.battery_soh < 90.0) color: #yellow;
                data "80-85" value: vehicle count (each.battery_soh >= 80.0 and each.battery_soh < 85.0) color: #orange;
                data "70-80" value: vehicle count (each.battery_soh >= 70.0 and each.battery_soh < 80.0) color: #darkorange;
                data "< 70" value: vehicle count (each.battery_soh < 70.0)                               color: #red;
            }
        }

        // TABLEAU DE BORD 3 — Stations & Performance
        display "Stations & Performance" refresh: every(1 #cycles) {

            chart "Occupation des stations (%)" type: histogram background: #white {
                loop s over: list(charging_station) {
                    data "St-" + int(s) value: s.occupancy_rate() * 100 color: #steelblue;
                }
            }

            chart "Véhicules servis par station" type: histogram background: #white {
                loop s over: list(charging_station) {
                    data "St-" + int(s) value: s.nb_served color: #limegreen;
                }
            }

            chart "File d'attente par station" type: histogram background: #white {
                loop s over: list(charging_station) {
                    data "St-" + int(s) value: length(s.queue) color: #orange;
                }
            }

            chart "KPIs de performance" type: series background: #white {
                data "Taux occ. moy. (%)"   value: kpi_avg_station_occ     color: #steelblue style: line;
                data "Attente moy. (cycles)" value: kpi_avg_wait_per_charge color: #orange    style: line;
                data "Débit recharges/100c"  value: kpi_throughput          color: #limegreen style: line;
            }
        }

        // TABLEAU DE BORD 4 — Mobilité & Fiabilité
        display "Mobilité & Fiabilité" refresh: every(1 #cycles) {

            chart "Distance totale parcourue (km)" type: series background: #white {
                data "Distance fleet" value: total_distance_km color: #dodgerblue style: line;
            }

            chart "Taux de panne (pannes / 100 km)" type: series background: #white {
                data "Taux panne" value: kpi_breakdown_rate color: #red style: line;
            }

            chart "Distance individuelle par véhicule (km)" type: histogram background: #white {
                loop v over: list(vehicle) {
                    data "V" + int(v) value: v.total_km color: #dodgerblue;
                }
            }

            
            // 50 véhicules, la file est rare. Ce graphique montre :
            //   - L'attente cumulée réelle (quand file active)
            //   - Le nb d'entrées en file (total_queue_entries)
            //   - L'attente moyenne par entrée en file
            chart "File d'attente — Statistiques" type: series background: #white {
                data "Attente cumulée (cycles)"  value: total_wait_time                                                                                    color: #darkorange style: line;
                data "Entrées en file (nb)"      value: float(total_queue_entries)                                                                         color: #purple     style: line;
                data "Attente moy./entrée"        value: (total_queue_entries > 0) ? (total_wait_time / float(total_queue_entries)) : 0.0  color: #red        style: line;
            }
        }

        // -------------------------------------------------------
        // MONITEURS — KPIs principaux (panneau latéral GAMA)
        // [Simulation]
        // -------------------------------------------------------
        monitor "── SIMULATION ──"          value: "Cycle " + cycle;
        monitor "Cycle courant"             value: cycle;
        monitor "Temps simulé (min)"        value: with_precision(cycle * step / 60.0, 1);
        monitor "Pas de temps (s)"          value: step;

        // [États flotte]
        monitor "── FLOTTE ──"             value: nb_vehicles;
        monitor "🚗 Roulant"               value: vehicle count (each.state = "driving");
        monitor "🔍 Recherche"             value: vehicle count (each.state = "searching");
        monitor "⏳ En file"               value: vehicle count (each.state = "queuing");
        monitor "⚡ En charge"             value: vehicle count (each.state = "charging");
        monitor "💥 En panne"              value: vehicle count (each.state = "broken");

        // [KPIs flotte]
        monitor "── KPIs FLOTTE ──"         value: "";
        monitor "Disponibilité (%)"         value: with_precision(kpi_fleet_availability, 1);
        monitor "Taux bloqués (%)"          value: with_precision(kpi_stuck_rate, 1);
        monitor "Taux panne /100km"         value: with_precision(kpi_breakdown_rate, 3);

        // [KPIs énergie]
        monitor "── ÉNERGIE ──"             value: "";
        monitor "Batterie moy. (%)"         value: with_precision(mean(vehicle collect each.battery_level), 1);
        monitor "Batterie min. (%)"         value: with_precision(kpi_min_battery, 1);
        monitor "SoH moyen (%)"             value: with_precision(mean(vehicle collect each.battery_soh), 1);
        monitor "Consommée (kWh)"           value: with_precision(total_consumed_kwh, 1);
        monitor "Rechargée (kWh)"           value: with_precision(total_energy_kwh, 1);
        monitor "Efficacité énergie"        value: with_precision(kpi_energy_efficiency, 2);

        // [KPIs stations]
        monitor "── STATIONS ──"            value: nb_stations;
        monitor "Occupation moy. (%)"       value: with_precision(kpi_avg_station_occ, 1);
        monitor "File totale (véhicules)"   value: kpi_total_queue_length;
        monitor "Attente moy. (cycles)"     value: with_precision(kpi_avg_wait_per_charge, 1);
        monitor "Débit (/100 cycles)"       value: with_precision(kpi_throughput, 2);

        // [Totaux]
        monitor "── TOTAUX ──"              value: "";
        monitor "Pannes totales"            value: total_breakdowns;
        monitor "Recharges totales"         value: total_charges;
        monitor "Récupérations"             value: total_recoveries;
        monitor "Distance fleet (km)"       value: with_precision(total_distance_km, 1);
    }
}

// =============================================================
//  EXPÉRIMENTATION BATCH
//  ALGORITHME 11 : Grid Search — Exploration exhaustive
//  Toutes les combinaisons des paramètres "among:" sont simulées
//  3 fois chacune (seeds différents).
// =============================================================
experiment BatchExploration type: batch repeat: 3 until: (cycle >= 5000) keep_seed: false {

    parameter "nb_vehicles"            var: nb_vehicles            among: [20, 50, 100, 200];
    parameter "nb_stations"            var: nb_stations            among: [5, 10, 15, 20];

    permanent {
        display "Résultats Batch" {
            chart "Pannes vs véhicules" type: scatter background: #white {
                data "Pannes" value: {float(nb_vehicles), float(total_breakdowns)} color: #red;
            }
            chart "Recharges vs stations" type: scatter background: #white {
                data "Recharges" value: {float(nb_stations), float(total_charges)} color: #green;
            }
            chart "Distance vs Nb Stations" type: scatter background: #white {
                data "Distance" value: {float(nb_stations), total_distance_km} color: #blue;
            }
        }
    }
}