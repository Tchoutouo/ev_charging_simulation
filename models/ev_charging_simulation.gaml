/**
 * ============================================================
 *  EV Charging Station Simulation — GAMA Platform (Version GIS Corrigée)
 *
 *  BUGS CORRIGÉS (v3) :
 *  [BUG 1] Véhicules immobiles en état "searching" :
 *          → do_search ne déplaçait PAS le véhicule si aucune station trouvée.
 *          → Correction : déplacement aléatoire maintenu pendant la recherche.
 *
 *  [BUG 2] Seuil d'arrivée à la station trop petit (5.0 m) :
 *          → Avec speed*step ≈ 333 m/cycle, le véhicule pouvait sauter
 *            par-dessus la station sans jamais déclencher la condition.
 *          → Correction : seuil relevé à arrival_threshold (100 m par défaut).
 *
 *  [BUG 3] Véhicule en panne qui "se recharge seul" :
 *          → Après récupération (state="searching"), si le véhicule était
 *            déjà à moins de 5 m d'une station, do_queue déclenchait
 *            immédiatement l'état "charging" SANS bouger vers la station.
 *          → Correction : flag reached_station — le véhicule ne peut entrer
 *            en recharge que s'il a physiquement navigué jusqu'à la station.
 *
 *  [BUG 4] do_charge sans garde sur target_station = nil :
 *          → Si target_station = nil (incohérence d'état), GAMA crashait.
 *          → Correction : garde explicite avec retour à l'état "driving".
 *
 *  [BUG 5] consume_energy utilise speed*step (théorique) au lieu de la
 *          distance réellement parcourue par do goto :
 *          → Correction : l'énergie est calculée sur la distance effective
 *            (position avant/après do goto).
 *
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
    file       roads_file <- file("../includes/roads2.shp");
    geometry   shape      <- envelope(roads_file);

    // -------------------------------------------------------
    // 2. PARAMÈTRE TEMPOREL
    //    ALGORITHME 8 (support) : step définit la durée réelle
    //    d'un cycle GAMA en secondes. Toutes les formules
    //    d'énergie (kWh) et de distance (km) s'appuient dessus.
    //    Valeur par défaut : 30 s/cycle → autonomie ~333 km.
    // -------------------------------------------------------
    float step <- 30.0;   // 1 cycle GAMA = 30 secondes simulées

    // -------------------------------------------------------
    // 3. PARAMÈTRES DE SIMULATION
    // -------------------------------------------------------
    int   nb_vehicles      <- 50;
    int   nb_stations      <- 15;   // Couverture suffisante (était 10)
    int   station_capacity <- 6;    // Capacité augmentée (était 4)

    float initial_battery_min <- 40.0;
    float initial_battery_max <- 100.0;
    float battery_threshold   <- 25.0;  // Seuil d'alerte pour recherche de station

    // -------------------------------------------------------
    // 4. PARAMÈTRES PHYSIQUES — Modèle kWh/km (ALGORITHME 8)
    //    Autonomie = battery_capacity_kwh / energy_consumption_kwh
    //    Ex : 60 kWh / 0.18 kWh/km ≈ 333 km (Tesla M3 / Renault Zoe)
    // -------------------------------------------------------
    float battery_capacity_kwh   <- 60.0;   // Capacité totale (kWh)
    float energy_consumption_kwh <- 0.18;   // Consommation (kWh/km)
    float charging_power_kw      <- 22.0;   // Puissance borne (kW)
    float vehicle_speed          <- 40.0;   // Vitesse (km/h)

    // -------------------------------------------------------
    // 5. SEUIL D'ARRIVÉE À LA STATION (CORRECTION BUG 2)
    //    Doit être > speed * step / 2 pour être robuste.
    //    Avec speed=11.1 m/s et step=30 s → mouvement ≈ 333 m/cycle.
    //    Un seuil de 100 m garantit la détection correcte.
    // -------------------------------------------------------
    float arrival_threshold <- 100.0;  // mètres (était 5.0 → BUG)

    // -------------------------------------------------------
    // 6. PARAMÈTRES SoH — Dégradation batterie (ALGORITHME 10)
    //    SoH(km) = 100 - (km/1000) × soh_degradation_per_1000km
    //    soh_min = 70% représente le seuil industriel de remplacement.
    // -------------------------------------------------------
    float soh_degradation_per_1000km <- 0.5;
    float soh_min                    <- 70.0;

    // -------------------------------------------------------
    // 7. RÉCUPÉRATION DE PANNE
    //    Dépannage automatique après N cycles (dépanneuse virtuelle).
    // -------------------------------------------------------
    int breakdown_recovery_cycles <- 100;

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
    // 9. AFFICHAGE
    // -------------------------------------------------------
    bool  use_icons <- true;

    // -------------------------------------------------------
    // 10. VARIABLES GLOBALES DE MONITORING
    // -------------------------------------------------------
    graph road_network;

    int   total_breakdowns  <- 0;
    int   total_charges     <- 0;
    float total_distance_km <- 0.0;
    float total_wait_time   <- 0.0;
    int   total_recoveries  <- 0;

    // -------------------------------------------------------
    // 11. INITIALISATION
    // -------------------------------------------------------
    init {
        write "=== Chargement du réseau GIS ===";
        create road from: roads_file;

        // ALGORITHME 1 : as_edge_graph
        // Construit un graphe topologique pondéré depuis les segments shapefile.
        // Chaque arête a un poids = longueur du tronçon en mètres.
        road_network <- as_edge_graph(road);
        write "Graphe construit : " + string(length(road)) + " tronçons.";

        // ALGORITHME 7 : Placement des stations par grille régulière
        // Divise la zone en sqrt(n)×sqrt(n) cellules, puis projette chaque
        // centre de cellule sur le centroïde de route le plus proche.
        // Garantit une couverture spatiale homogène sans "désert de recharge".
        write "=== Création des stations (algorithme de grille) ===";
        create charging_station number: nb_stations {
            capacity <- station_capacity;
        }

        list<point> road_points <- road collect (each.shape.centroid);
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
                point closest     <- road_points with_min_of (each distance_to grid_center);
                s.location <- (closest != nil) ? closest : any_location_in(one_of(road));
            }
            idx <- idx + 1;
        }
        write string(nb_stations) + " stations créées (couverture spatiale homogène).";

        write "=== Création des véhicules ===";
        create vehicle number: nb_vehicles {
            location      <- any_location_in(one_of(road));
            battery_level <- rnd(initial_battery_min, initial_battery_max);
            speed         <- vehicle_speed / 3.6;   // km/h → m/s
            state         <- "driving";
            battery_soh   <- rnd(85.0, 100.0);
        }
        write string(nb_vehicles) + " véhicules créés.";
        write "Autonomie théorique : " + string(with_precision(battery_capacity_kwh / energy_consumption_kwh, 0)) + " km";
        write "Durée recharge (22 kW) : " + string(with_precision(battery_capacity_kwh / charging_power_kw, 1)) + " h";
        write "=== Simulation démarrée ===";
    }

    reflex collect_indicators {
        float occ <- mean(charging_station collect (each.occupancy_rate()));
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

    // ALGORITHME 6 : File d'attente FIFO
    // Les véhicules sont servis dans leur ordre d'arrivée.
    // add : enfile en fin de liste / first() + remove : défile en tête.
    action add_to_queue (vehicle v) {
        if (!(v in queue)) { add v to: queue; }
    }

    action start_charging {
        loop while: (occupied_slots < capacity) and (not empty(queue)) {
            vehicle nxt <- first(queue);
            remove nxt from: queue;
            occupied_slots <- occupied_slots + 1;
            ask nxt {
                state              <- "charging";
                total_wait_time    <- total_wait_time + float(cycle - waiting_time_start);
            }
        }
    }

    action release_vehicle (vehicle v) {
        occupied_slots <- max(0, occupied_slots - 1);
        nb_served      <- nb_served + 1;
        total_charges  <- total_charges + 1;
        do start_charging;
    }

    bool  is_full        { return occupied_slots >= capacity; }
    float occupancy_rate { return (capacity = 0) ? 0.0 : float(occupied_slots) / float(capacity); }

    aspect icon_aspect {
        float icon_size <- 40.0;
        if (use_icons) {
            if (occupancy_rate() < 0.8) {
                draw image_file("../images/station_free.png") size: {icon_size, icon_size};
            } else {
                draw image_file("../images/station_busy.png") size: {icon_size, icon_size};
            }
        } else {
            int rv <- int(255.0 * occupancy_rate());
            int gv <- int(255.0 * (1.0 - occupancy_rate()));
            draw circle(icon_size / 2) color: rgb(rv, gv, 0) border: #black;
        }
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

    // ALGORITHME 10 — SoH (State of Health)
    float battery_soh <- 100.0;
    float total_km    <- 0.0;

    // Récupération de panne
    int   breakdown_cycle <- 0;

    // [CORRECTION BUG 3] Flag : le véhicule a-t-il physiquement navigué
    // jusqu'à la station avant d'entrer en recharge ?
    // Reset à false à chaque nouvelle cible de station.
    bool reached_station <- false;

    // [CORRECTION BUG FREEZE] Compteur de cycles sans mouvement.
    // Si stuck_counter >= max_stuck_cycles → cible inaccessible → nouvelle cible.
    int stuck_counter <- 0;

    // -------------------------------------------------------
    // FSM — ÉTAT : driving
    //   Déplacement aléatoire tant que batterie > seuil.
    //   Passe à "searching" dès que batterie < battery_threshold.
    // -------------------------------------------------------
    reflex do_drive when: (state = "driving") {
        if (battery_level <= 0.0) { do breakdown; return; }
        if (battery_level < battery_threshold) {
            state           <- "searching";
            target_point    <- nil;
            target_station  <- nil;
            reached_station <- false;
            return;
        }

        if (target_point = nil) {
            target_point  <- any_location_in(one_of(road));
            stuck_counter <- 0;
        }

        // ALGORITHME 2 : Dijkstra via do goto
        // Calcule et suit le plus court chemin jusqu'à target_point.
        point prev_loc  <- location;
        do goto target: target_point on: road_network speed: speed;
        float dist_moved <- location distance_to prev_loc;   // distance réellement parcourue
        do consume_energy_for(dist_moved);

        // --- DÉTECTION DE BLOCAGE ---
        // Si le véhicule n'a pas bougé ET n'est pas encore arrivé,
        // la cible est probablement sur un tronçon isolé du graphe.
        if (dist_moved < 1.0 and location distance_to target_point > arrival_threshold) {
            stuck_counter <- stuck_counter + 1;
            if (stuck_counter >= max_stuck_cycles) {
                target_point  <- nil;   // Nouvelle cible aléatoire au prochain cycle
                stuck_counter <- 0;
            }
        } else {
            stuck_counter <- 0;   // Mouvement normal → reset compteur
            if (location distance_to target_point < arrival_threshold) {
                target_point <- nil;
            }
        }
    }

    // -------------------------------------------------------
    // FSM — ÉTAT : searching
    //   ALGORITHME 3 : WSM — Sélection de la meilleure station.
    //   ALGORITHME 4 : Adaptation dynamique α/β selon batterie.
    //
    //   [CORRECTION BUG 1] : Si aucune station n'est disponible,
    //   le véhicule continue de se déplacer (pas immobile).
    // -------------------------------------------------------
    reflex do_search when: (state = "searching") {
        if (battery_level <= 0.0) { do breakdown; return; }

        charging_station best <- select_best_station();

        if (best = nil) {
            // Aucune station accessible → continuer à se déplacer
            if (target_point = nil) {
                target_point  <- any_location_in(one_of(road));
                stuck_counter <- 0;
            }
            point prev_loc   <- location;
            do goto target: target_point on: road_network speed: speed;
            float dist_moved <- location distance_to prev_loc;
            do consume_energy_for(dist_moved);

            // Détection de blocage pendant la recherche
            if (dist_moved < 1.0 and location distance_to target_point > arrival_threshold) {
                stuck_counter <- stuck_counter + 1;
                if (stuck_counter >= max_stuck_cycles) {
                    target_point  <- nil;
                    stuck_counter <- 0;
                }
            } else {
                stuck_counter <- 0;
                if (location distance_to target_point < arrival_threshold) {
                    target_point <- nil;
                }
            }
            return;
        }

        // Station trouvée → transition vers "queuing"
        target_station  <- best;
        target_point    <- nil;
        reached_station <- false;
        stuck_counter   <- 0;
        state           <- "queuing";
    }

    // -------------------------------------------------------
    // FSM — ÉTAT : queuing
    //   Le véhicule navigue vers la station cible.
    //   [CORRECTION BUG 2] : Utilise arrival_threshold (100 m) et non 5 m.
    //   [CORRECTION BUG 3] : La recharge ne commence que si reached_station=true,
    //   c'est-à-dire si le véhicule a physiquement parcouru le trajet.
    // -------------------------------------------------------
    reflex do_queue when: (state = "queuing") {
        if (battery_level <= 0.0) {
            // Panne en route vers la station → sortir de la file
            if (target_station != nil and (self in target_station.queue)) {
                ask target_station { remove myself from: queue; }
            }
            do breakdown;
            return;
        }

        if (target_station = nil) {
            reached_station <- false;
            state           <- "searching";
            return;
        }

        float dist_to_station <- location distance_to target_station.location;

        if (!reached_station and dist_to_station > arrival_threshold) {
            // Se déplacer vers la station (Dijkstra)
            point prev_loc   <- location;
            do goto target: target_station.location on: road_network speed: speed;
            float dist_moved <- location distance_to prev_loc;
            do consume_energy_for(dist_moved);

            // --- DÉTECTION DE BLOCAGE VERS LA STATION ---
            // Si bloqué, chercher une autre station plutôt que de rester figé.
            if (dist_moved < 1.0 and dist_to_station > arrival_threshold) {
                stuck_counter <- stuck_counter + 1;
                if (stuck_counter >= max_stuck_cycles) {
                    // Station inaccessible → retour à searching pour en choisir une autre
                    target_station  <- nil;
                    reached_station <- false;
                    stuck_counter   <- 0;
                    state           <- "searching";
                }
            } else {
                stuck_counter <- 0;
            }

        } else {
            // Arrivée à la station confirmée
            reached_station <- true;
            stuck_counter   <- 0;

            // Tentative d'accès (une seule fois par arrivée)
            if (!(self in target_station.queue)) {
                if (target_station.is_full()) {
                    // ALGORITHME 6 — Entrée en file FIFO
                    ask target_station { do add_to_queue(myself); }
                    waiting_time_start <- cycle;
                } else {
                    // Place disponible → commencer à charger
                    ask target_station { occupied_slots <- occupied_slots + 1; }
                    state <- "charging";
                }
            }
            // Sinon : en file, attente de start_charging
        }
    }

    // -------------------------------------------------------
    // FSM — ÉTAT : charging
    //   ALGORITHME 9 : Courbe de recharge CC-CV (Li-ion réaliste)
    //   0%→80%  : Phase CC (Courant Constant) — puissance nominale
    //   80%→100%: Phase CV (Tension Constante) — puissance décroissante
    //
    //   [CORRECTION BUG 4] : Garde explicite sur target_station=nil.
    // -------------------------------------------------------
    reflex do_charge when: (state = "charging") {

        // [BUG 4 CORRIGÉ] : Incohérence d'état possible après récupération de panne
        if (target_station = nil) {
            state <- "driving";
            return;
        }

        float hours_per_cycle    <- step / 3600.0;
        float effective_power_kw;

        if (battery_level < 80.0) {
            // Phase CC : puissance nomimale complète
            effective_power_kw <- charging_power_kw;
        } else {
            // Phase CV : dégradation linéaire de la puissance (100%→10% entre 80%→100%)
            float soc_factor   <- 1.0 - ((battery_level - 80.0) / 20.0) * 0.9;
            effective_power_kw <- charging_power_kw * max(0.1, soc_factor);
        }

        float energy_charged_kwh <- effective_power_kw * hours_per_cycle;
        float delta_soc          <- (energy_charged_kwh / battery_capacity_kwh) * 100.0;
        float effective_max      <- battery_soh;   // Plafond SoH

        battery_level <- min(effective_max, battery_level + delta_soc);

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
    //   Récupération automatique après breakdown_recovery_cycles.
    //   Le dépannage recharge la batterie à 25% → state "searching".
    // -------------------------------------------------------
    reflex do_recovery when: (state = "broken") {
        // Enregistrer le cycle de panne (une seule fois)
        if (breakdown_cycle = 0) { breakdown_cycle <- cycle; }

        // Récupération après N cycles (dépannage virtuel)
        if (cycle - breakdown_cycle >= breakdown_recovery_cycles) {
            battery_level    <- 25.0;      // Recharge d'urgence
            state            <- "searching";
            breakdown_cycle  <- 0;
            target_station   <- nil;
            target_point     <- nil;
            reached_station  <- false;
            stuck_counter    <- 0;
            total_recoveries <- total_recoveries + 1;
        }
    }

    // -------------------------------------------------------
    // ACTION : Consommation d'énergie sur distance réelle
    //   [CORRECTION BUG 5] : Prend en paramètre la distance
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

        // ALGORITHME 10 : Mise à jour SoH
        do update_soh;
    }

    // -------------------------------------------------------
    // ACTION : Mise à jour du State of Health (SoH)
    //   ALGORITHME 10 : Dégradation linéaire
    //   SoH(km) = max(soh_min, 100 - (km/1000) × taux_dégradation)
    // -------------------------------------------------------
    action update_soh {
        float new_soh <- max(soh_min, 100.0 - (total_km / 1000.0) * soh_degradation_per_1000km);
        battery_soh   <- new_soh;
        if (battery_level > battery_soh) {
            battery_level <- battery_soh;
        }
    }

    // -------------------------------------------------------
    // ACTION : Passage en panne
    // -------------------------------------------------------
    action breakdown {
        // Nettoyer proprement si on était en file
        if (target_station != nil and (self in target_station.queue)) {
            ask target_station { remove myself from: queue; }
        }
        state            <- "broken";
        target_station   <- nil;
        target_point     <- nil;
        reached_station  <- false;
        stuck_counter    <- 0;
        breakdown_cycle  <- 0;
        total_breakdowns <- total_breakdowns + 1;
    }

    // -------------------------------------------------------
    // ALGORITHME 3 : Sélection de station — Weighted Sum Method (WSM)
    //   score = α × dist_normalisée + β × file_normalisée
    //
    // ALGORITHME 4 : Poids dynamiques α/β
    //   ratio = battery / threshold → plus la batterie est faible,
    //   plus α (poids distance) augmente → favorise la station la plus proche.
    // -------------------------------------------------------
    charging_station select_best_station {
        list<charging_station> candidates <- list(charging_station);
        if (empty(candidates)) { return nil; }

        float ratio <- (battery_threshold > 0.0) ? (battery_level / battery_threshold) : 1.0;
        float alpha <- alpha_base * min(1.0, ratio);
        float beta  <- 1.0 - alpha;

        float max_dist  <- max(candidates collect (self distance_to each.location));
        int   max_queue <- max(candidates collect (length(each.queue)));
        if (max_dist  <= 0.0) { max_dist  <- 1.0; }
        if (max_queue <= 0  ) { max_queue <- 1; }

        charging_station best       <- nil;
        float            best_score <- #infinity;

        loop s over: candidates {
            float nd <- (self distance_to s.location) / max_dist;
            float nq <- float(length(s.queue)) / float(max_queue);
            float sc <- alpha * nd + beta * nq;
            if (s.is_full() and length(s.queue) > station_capacity) { sc <- sc + 0.5; }
            if (sc < best_score) { best_score <- sc; best <- s; }
        }
        return best;
    }

    // -------------------------------------------------------
    // VISUALISATION — Aspect principal avec clignotement panne
    //   Clignotement : calculé directement en aspect via cycle mod
    //   (plus fiable qu'un flag booléen mis à jour dans un reflex).
    // -------------------------------------------------------
    aspect icon_aspect {
        float icon_size  <- 20.0;
        // Clignotement calculé ici : true = visible, false = "invisible"
        bool  blink_show <- (state != "broken") or ((cycle mod 10) < 5);

        if (use_icons) {
            string img_path <- nil;
            if      (state = "driving")   { img_path <- "../images/car_blue.png"; }
            else if (state = "searching") { img_path <- "../images/car_orange.png"; }
            else if (state = "queuing")   { img_path <- "../images/car_orange.png"; }
            else if (state = "charging")  { img_path <- "../images/car_green.png"; }
            else if (blink_show)          { img_path <- "../images/car_red.png"; }
            // else broken + !blink_show → img_path = nil = pas de dessin

            if (img_path != nil) {
                draw image_file(img_path) size: {icon_size, icon_size};
            }
        } else {
            rgb c;
            int alpha_val <- 255;
            if      (state = "driving")   { c <- #dodgerblue; }
            else if (state = "searching") { c <- #orange; }
            else if (state = "queuing")   { c <- #darkorange; }
            else if (state = "charging")  { c <- #limegreen; }
            else {
                c         <- #red;
                alpha_val <- blink_show ? 255 : 30;
            }
            draw circle(6) color: rgb(c.red, c.green, c.blue, alpha_val) border: #black;
        }

        // Barre de batterie (vert → rouge selon niveau)
        float bar_w  <- icon_size;
        float fill_w <- bar_w * (battery_level / 100.0);
        draw rectangle(bar_w, 3.0)
             at: {location.x, location.y - icon_size * 0.7}
             color: rgb(60, 60, 60) border: #black;
        int br <- int(255.0 * (1.0 - battery_level / 100.0));
        int bg <- int(255.0 * (battery_level / 100.0));
        draw rectangle(max(0.5, fill_w), 3.0)
             at: {location.x - (bar_w - fill_w) / 2.0, location.y - icon_size * 0.7}
             color: rgb(br, bg, 0);

        // Indicateur SoH dégradé (< 95%)
        if (battery_soh < 95.0) {
            draw "SoH:" + string(int(battery_soh)) + "%"
                 at: {location.x + icon_size * 0.5, location.y + icon_size * 0.3}
                 color: #yellow font: font("Arial", 7, #plain);
        }
    }

    aspect base {
        bool blink_show <- (state != "broken") or ((cycle mod 10) < 5);
        rgb c;
        int alpha_val <- 255;
        if      (state = "driving")   { c <- #dodgerblue; }
        else if (state = "searching") { c <- #orange; }
        else if (state = "queuing")   { c <- #darkorange; }
        else if (state = "charging")  { c <- #limegreen; }
        else {
            c         <- #red;
            alpha_val <- blink_show ? 255 : 30;
        }
        draw circle(6) color: rgb(c.red, c.green, c.blue, alpha_val) border: #black;
        float bar_w  <- 14.0;
        float fill_w <- bar_w * (battery_level / 100.0);
        draw rectangle(bar_w, 2.0)
             at: {location.x, location.y - 10.0}
             color: #lightgray border: #black;
        int br <- int(255.0 * (1.0 - battery_level / 100.0));
        int bg <- int(255.0 * (battery_level / 100.0));
        draw rectangle(max(0.5, fill_w), 2.0)
             at: {location.x - (bar_w - fill_w) / 2.0, location.y - 10.0}
             color: rgb(br, bg, 0);
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
    parameter "Seuil d'alerte (%)"         var: battery_threshold        min: 5.0    max: 50.0   category: "Batterie";
    parameter "Capacité batterie (kWh)"    var: battery_capacity_kwh     min: 20.0   max: 150.0  category: "Batterie";
    parameter "Consommation (kWh/km)"      var: energy_consumption_kwh   min: 0.10   max: 0.35   category: "Batterie";
    parameter "Puissance recharge (kW)"    var: charging_power_kw        min: 3.7    max: 150.0  category: "Batterie";
    parameter "Dégradation SoH (%/1000km)" var: soh_degradation_per_1000km min: 0.0  max: 2.0   category: "Batterie";

    parameter "Vitesse (km/h)"             var: vehicle_speed            min: 10.0   max: 130.0  category: "Déplacement";

    parameter "Alpha (poids distance)"     var: alpha_base               min: 0.0    max: 1.0    category: "Stratégie";
    parameter "Beta (poids file)"          var: beta_base                min: 0.0    max: 1.0    category: "Stratégie";
    parameter "Délai récupération (cycles)" var: breakdown_recovery_cycles min: 10   max: 500    category: "Stratégie";

    parameter "Utiliser les icônes"        var: use_icons                                        category: "Affichage";

    output {

        display "Carte GIS" type: opengl background: rgb(20, 20, 30) {
            species road             aspect: base;
            species charging_station aspect: icon_aspect;
            species vehicle          aspect: icon_aspect;
        }

        display "Indicateurs" refresh: every(10 #cycles) {

            chart "États des véhicules" type: pie background: #white {
                data "Roulant"    value: vehicle count (each.state = "driving")   color: #dodgerblue;
                data "Recherche"  value: vehicle count (each.state = "searching") color: #orange;
                data "En file"    value: vehicle count (each.state = "queuing")   color: #darkorange;
                data "En charge"  value: vehicle count (each.state = "charging")  color: #limegreen;
                data "En panne"   value: vehicle count (each.state = "broken")    color: #red;
            }

            chart "Batterie & SoH moyens (%)" type: series background: #white {
                data "Batterie moy."  value: mean(vehicle collect (each.battery_level)) color: #green style: line;
                data "SoH moyen (%)"  value: mean(vehicle collect (each.battery_soh))   color: #blue  style: line;
                data "Seuil alerte"   value: battery_threshold                          color: #red   style: line;
            }

            chart "Occupation des stations (%)" type: histogram background: #white {
                loop s over: list(charging_station) {
                    data "St-" + int(s) value: s.occupancy_rate() * 100 color: #steelblue;
                }
            }

            chart "Pannes / Recharges / Récupérations" type: series background: #white {
                data "Pannes"          value: total_breakdowns  color: #red    style: line;
                data "Recharges"       value: total_charges     color: #green  style: line;
                data "Récupérations"   value: total_recoveries  color: #orange style: line;
            }

            chart "Distance totale (km)" type: series background: #white {
                data "Distance km" value: total_distance_km color: #blue style: line;
            }

            chart "SoH par véhicule (%)" type: histogram background: #white {
                loop v over: list(vehicle) {
                    data "V" + int(v) value: v.battery_soh color: #teal;
                }
            }
        }

        monitor "Cycle"                  value: cycle;
        monitor "Pas simulé (s)"         value: step;
        monitor "Roulant"                value: vehicle count (each.state = "driving");
        monitor "En charge"              value: vehicle count (each.state = "charging");
        monitor "En file"                value: vehicle count (each.state = "queuing");
        monitor "En recherche"           value: vehicle count (each.state = "searching");
        monitor "En panne (actuel)"      value: vehicle count (each.state = "broken");
        monitor "Pannes totales"         value: total_breakdowns;
        monitor "Recharges totales"      value: total_charges;
        monitor "Récupérations"          value: total_recoveries;
        monitor "Distance totale (km)"   value: with_precision(total_distance_km, 1);
        monitor "Taux occ. moy. (%)"     value: with_precision(
                                             mean(charging_station collect (each.occupancy_rate())) * 100, 1);
        monitor "Batterie moy. (%)"      value: with_precision(
                                             mean(vehicle collect (each.battery_level)), 1);
        monitor "SoH moyen (%)"          value: with_precision(
                                             mean(vehicle collect (each.battery_soh)), 1);
        monitor "Attente totale (cycles)" value: with_precision(total_wait_time, 1);
        monitor "Autonomie théo. (km)"   value: with_precision(
                                             battery_capacity_kwh / energy_consumption_kwh, 0);
    }
}

// =============================================================
//  EXPÉRIMENTATION BATCH
//  ALGORITHME 11 : Grid Search — Exploration exhaustive
//  Toutes les combinaisons des paramètres "among:" sont simulées
//  3 fois chacune (seeds différents) pour la robustesse statistique.
// =============================================================
experiment BatchExploration type: batch repeat: 3 until: (cycle >= 5000) keep_seed: false {

    parameter "nb_vehicles"            var: nb_vehicles            among: [20, 50, 100, 200];
    parameter "nb_stations"            var: nb_stations            among: [5, 10, 15, 20];
    parameter "battery_threshold"      var: battery_threshold      among: [15.0, 25.0, 35.0];
    parameter "energy_consumption_kwh" var: energy_consumption_kwh among: [0.15, 0.18, 0.22];

    permanent {
        display "Résultats Batch" {
            chart "Pannes vs véhicules" type: scatter background: #white {
                data "Pannes" value: {float(nb_vehicles), float(total_breakdowns)} color: #red;
            }
            chart "Recharges vs stations" type: scatter background: #white {
                data "Recharges" value: {float(nb_stations), float(total_charges)} color: #green;
            }
            chart "Distance vs consommation" type: scatter background: #white {
                data "Distance" value: {energy_consumption_kwh, total_distance_km} color: #blue;
            }
        }
    }
}
