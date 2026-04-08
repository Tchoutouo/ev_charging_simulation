/**
 * ============================================================
 *  EV Charging Station Simulation — GAMA Platform (Version GIS Améliorée)
 *  Modèle multi-agents avec GIS, graphe routier, FSM, files d'attente,
 *  modèle de consommation réaliste (kWh/km), dégradation batterie (SoH),
 *  courbe de recharge CC-CV, récupération de panne, et clignotement visuel.
 *
 *  Basé sur : GAMA 1.9.x — Skill Moving + GIS shapefile
 *
 *  Fichiers requis dans includes/ :
 *    - roads2.shp  (+ .dbf .prj .shx)  : réseau routier
 *
 *  Images requises dans images/ :
 *    - car_blue.png       véhicule en circulation
 *    - car_orange.png     véhicule en recherche de station
 *    - car_green.png      véhicule en recharge
 *    - car_red.png        véhicule en panne
 *    - station_free.png   station disponible
 *    - station_busy.png   station occupée/saturée
 *
 * ============================================================
 *
 *  ALGORITHMES UTILISÉS (résumé) :
 *   1. as_edge_graph()          → Graphe topologique depuis shapefile
 *   2. do goto / Dijkstra       → Plus court chemin sur graphe routier
 *   3. WSM + Score dynamique    → Sélection de station multi-critères
 *   4. Ratio linéaire α/β       → Adaptation urgence (fuzzy-like)
 *   5. FSM déterministe (DFA)   → Contrôle des états du véhicule
 *   6. File FIFO                → Gestion attente à la station
 *   7. Grille + voisin + road   → Placement spatial des stations
 *   8. Modèle physique kWh/km   → Consommation d'énergie réaliste
 *   9. CC-CV                    → Courbe de recharge Li-ion réaliste
 *  10. SoH linéaire             → Dégradation batterie selon km
 *  11. Grid Search (batch)      → Exploration exhaustive paramètres
 *
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
    //    ALGORITHME : Chaque cycle GAMA représente `step` secondes
    //    de temps simulé (ici 30 secondes = 0.5 minute).
    //    Cela garantit la cohérence des unités physiques :
    //    distance (m), vitesse (m/s), énergie (kWh).
    // -------------------------------------------------------
    float step <- 30.0;   // 1 cycle = 30 secondes simulées

    // -------------------------------------------------------
    // 3. PARAMÈTRES DE SIMULATION (sliders dans l'experiment)
    // -------------------------------------------------------
    int   nb_vehicles      <- 50;
    int   nb_stations      <- 15;   // Augmenté (était 10)
    int   station_capacity <- 6;    // Augmenté (était 4)

    float initial_battery_min <- 40.0;   // Augmenté (était 30%)
    float initial_battery_max <- 100.0;
    float battery_threshold   <- 25.0;   // Seuil d'alerte batterie (%)

    // -------------------------------------------------------
    // 4. PARAMÈTRES PHYSIQUES — Modèle de consommation réaliste
    //    ALGORITHME 8 : Modèle physique kWh/km
    //    Inspiré des données réelles de VE (Renault Zoe, Tesla M3).
    //    Autonomie théorique = battery_capacity_kwh / energy_consumption_kwh
    //    Ex : 60 kWh / 0.18 kWh/km ≈ 333 km d'autonomie
    // -------------------------------------------------------
    float battery_capacity_kwh    <- 60.0;    // Capacité batterie (kWh) — ex: Tesla Model 3
    float energy_consumption_kwh  <- 0.18;    // Consommation (kWh/km) — moyenne VE : 0.15-0.22
    float charging_power_kw       <- 22.0;    // Puissance de recharge (kW) — borne AC 22kW
    float vehicle_speed           <- 40.0;    // Vitesse de croisière (km/h)

    // -------------------------------------------------------
    // 5. PARAMÈTRES DÉGRADATION BATTERIE (State of Health)
    //    ALGORITHME 10 : SoH linéaire
    //    Perte de SoH proportionnelle aux km parcourus.
    //    Le SoH représente la capacité restante par rapport à l'origine.
    //    SoH min = 70% (seuil de remplacement batterie industrie)
    // -------------------------------------------------------
    float soh_degradation_per_1000km <- 0.5;  // Perte de 0.5% de SoH / 1000 km
    float soh_min                    <- 70.0; // SoH minimum (70% = remplacement batterie)

    // -------------------------------------------------------
    // 6. PARAMÈTRES RÉCUPÉRATION DE PANNE
    //    Durée en cycles avant qu'un véhicule en panne soit récupéré
    //    par un service de dépannage (recharge d'urgence à 25%).
    // -------------------------------------------------------
    int breakdown_recovery_cycles <- 100;  // ~50 min simulées avant dépannage

    // -------------------------------------------------------
    // 7. PARAMÈTRES STRATÉGIE (scoring station)
    // -------------------------------------------------------
    float alpha_base <- 0.7;
    float beta_base  <- 0.3;

    // -------------------------------------------------------
    // 8. AFFICHAGE
    // -------------------------------------------------------
    bool  use_icons <- true;

    // -------------------------------------------------------
    // 9. VARIABLES GLOBALES DE MONITORING
    // -------------------------------------------------------
    graph road_network;

    int   total_breakdowns   <- 0;
    int   total_charges      <- 0;
    float total_distance_km  <- 0.0;   // Distance totale en km
    float total_wait_time    <- 0.0;
    int   total_recoveries   <- 0;     // Nb de véhicules récupérés après panne

    // -------------------------------------------------------
    // 10. INITIALISATION
    // -------------------------------------------------------
    init {
        write "=== Chargement du réseau GIS ===";
        create road from: roads_file;
        // ALGORITHME 1 : as_edge_graph — Construction d'un graphe topologique
        // Chaque segment shapefile devient une arête pondérée (poids = longueur en m).
        // Le graphe est non-orienté et utilisé pour le routage par Dijkstra.
        road_network <- as_edge_graph(road);
        write "Graphe construit : " + string(length(road)) + " tronçons.";

        write "=== Création des stations (placement spatial intelligent) ===";
        // ALGORITHME 7 : Placement par grille + projection sur route la plus proche
        // Principe : diviser la zone en une grille régulière (sqrt(n) x sqrt(n)),
        // puis pour chaque cellule trouver le point de route le plus proche du centre.
        // Garantit une couverture spatiale homogène (évite les "déserts de recharge").
        create charging_station number: nb_stations {
            capacity <- station_capacity;
        }

        list<point> road_points <- road collect (each.shape.centroid);
        int grid_side <- int(sqrt(float(nb_stations))) + 1;
        float dw <- world.shape.width  / float(grid_side);
        float dh <- world.shape.height / float(grid_side);
        float ox  <- world.location.x - world.shape.width  / 2.0;
        float oy  <- world.location.y - world.shape.height / 2.0;

        int idx <- 0;
        loop s over: list(charging_station) {
            if (idx < nb_stations) {
                int row <- int(idx / grid_side);
                int col <- idx mod grid_side;
                point grid_center <- {ox + (col + 0.5) * dw, oy + (row + 0.5) * dh};
                // Projection : trouver le nœud routier le plus proche du centre de grille
                point closest <- road_points with_min_of (each distance_to grid_center);
                s.location <- closest != nil ? closest : any_location_in(one_of(road));
            }
            idx <- idx + 1;
        }
        write string(nb_stations) + " stations créées (couverture spatiale homogène).";

        write "=== Création des véhicules ===";
        create vehicle number: nb_vehicles {
            location      <- any_location_in(one_of(road));
            battery_level <- rnd(initial_battery_min, initial_battery_max);
            // ALGORITHME 8 : Conversion vitesse km/h → m/s pour GAMA
            speed         <- vehicle_speed / 3.6;
            state         <- "driving";
            battery_soh   <- rnd(85.0, 100.0);  // SoH initial entre 85 et 100%
        }
        write string(nb_vehicles) + " véhicules créés.";
        write "=== Simulation démarrée ===";
        write "  Autonomie théorique : " + string(with_precision(battery_capacity_kwh / energy_consumption_kwh, 0)) + " km";
        write "  Durée recharge complète (22kW) : " + string(with_precision(battery_capacity_kwh / charging_power_kw, 1)) + " h";
    }

    // -------------------------------------------------------
    // 11. COLLECTE DES INDICATEURS (chaque cycle)
    // -------------------------------------------------------
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
    float         total_wait     <- 0.0;
    int           nb_served      <- 0;

    // ALGORITHME 6 : File FIFO (First-In, First-Out)
    // Les véhicules sont servis dans leur ordre d'arrivée à la station.
    // La structure list<vehicle> agit comme une file : add en queue, first() en tête.
    action add_to_queue (vehicle v) {
        if (!(v in queue)) { add v to: queue; }
    }

    action start_charging {
        // Dépile les véhicules en attente si des places se libèrent (FIFO)
        loop while: (occupied_slots < capacity) and (not empty(queue)) {
            vehicle nxt <- first(queue);
            remove nxt from: queue;
            occupied_slots <- occupied_slots + 1;
            ask nxt {
                state <- "charging";
                total_wait_time <- total_wait_time + float(cycle - waiting_time_start);
            }
        }
    }

    action release_vehicle (vehicle v) {
        occupied_slots <- max(0, occupied_slots - 1);
        nb_served      <- nb_served + 1;
        total_charges  <- total_charges + 1;
        do start_charging;
    }

    bool is_full         { return occupied_slots >= capacity; }
    float occupancy_rate { return (capacity = 0) ? 0.0 : float(occupied_slots) / float(capacity); }

    // -------------------------------------------------------
    // VISUALISATION : icône + infos dynamiques
    // -------------------------------------------------------
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
//  ESPÈCE : vehicle  (Skill Moving pour déplacement GIS)
//
//  ALGORITHME 5 : Automate à états finis déterministe (DFA/FSM)
//  États : driving → searching → queuing → charging → driving
//          (+ broken, état transitoire avec récupération)
//
//  Transitions :
//    driving   → searching : battery < battery_threshold
//    driving   → broken    : battery = 0
//    searching → queuing   : station sélectionnée (WSM)
//    queuing   → charging  : place libre / file libérée
//    queuing   → broken    : battery = 0 en route
//    charging  → driving   : battery ≥ SoH * 0.95
//    broken    → searching : après breakdown_recovery_cycles cycles (dépannage)
// =============================================================
species vehicle skills: [moving] {

    float            battery_level;
    string           state;
    charging_station target_station    <- nil;
    point            target_point      <- nil;
    float            distance_traveled <- 0.0;  // en mètres
    int              waiting_time_start <- 0;

    // ALGORITHME 10 : Dégradation batterie (State of Health)
    float battery_soh       <- 100.0;   // 100% = batterie neuve
    float total_km          <- 0.0;     // km parcourus (pour calcul SoH)

    // Récupération de panne
    int   breakdown_cycle   <- 0;       // cycle où la panne s'est produite

    // Compteur de clignotement (visuel uniquement)
    bool  blink_visible     <- true;

    // -------------------------------------------------------
    // FSM — ÉTAT : driving
    // -------------------------------------------------------
    reflex do_drive when: (state = "driving") {
        if (battery_level <= 0.0) { do breakdown; return; }
        if (battery_level < battery_threshold) {
            state        <- "searching";
            target_point <- nil;
            return;
        }

        // Nouvelle destination aléatoire si arrivé ou première fois
        if (target_point = nil) {
            target_point <- any_location_in(one_of(road));
        }

        // ALGORITHME 2 : Routage par Dijkstra (skill moving - do goto)
        // GAMA calcule le plus court chemin (en distance) entre la position
        // actuelle et target_point en utilisant le graphe road_network.
        // L'algorithme de Dijkstra garantit l'optimalité du chemin.
        do goto target: target_point on: road_network speed: speed;
        do consume_energy;

        if (location distance_to target_point < 5.0) {
            target_point <- nil;
        }
    }

    // -------------------------------------------------------
    // FSM — ÉTAT : searching
    //   ALGORITHME 3 : Sélection multi-critères WSM (Weighted Sum Method)
    //   + ALGORITHME 4 : Adaptation dynamique des poids α/β
    // -------------------------------------------------------
    reflex do_search when: (state = "searching") {
        if (battery_level <= 0.0) { do breakdown; return; }
        charging_station best <- select_best_station();
        if (best = nil) { return; }
        target_station <- best;
        target_point   <- target_station.location;
        state          <- "queuing";
    }

    // -------------------------------------------------------
    // FSM — ÉTAT : queuing
    // -------------------------------------------------------
    reflex do_queue when: (state = "queuing") {
        if (battery_level <= 0.0) {
            if (target_station != nil) {
                ask target_station { remove myself from: queue; }
            }
            do breakdown;
            return;
        }

        if (target_station = nil) { state <- "searching"; return; }

        if (location distance_to target_station.location > 5.0) {
            // ALGORITHME 2 — Dijkstra encore utilisé pour naviguer vers la station
            do goto target: target_station.location on: road_network speed: speed;
            do consume_energy;
        } else {
            if (!(self in target_station.queue)) {
                if (target_station.is_full()) {
                    // ALGORITHME 6 — Entrée dans la file FIFO
                    ask target_station { do add_to_queue(myself); }
                    waiting_time_start <- cycle;
                } else {
                    ask target_station { occupied_slots <- occupied_slots + 1; }
                    state <- "charging";
                }
            }
        }
    }

    // -------------------------------------------------------
    // FSM — ÉTAT : charging
    //   ALGORITHME 9 : Courbe de recharge CC-CV (Li-ion réaliste)
    //   Phase CC (Courant Constant) : 0% → 80%  — charge rapide
    //   Phase CV (Tension Constante) : 80% → 100% — charge ralentie
    //   Modélise le comportement réel des chargeurs de VE.
    //   Puissance effective : P(t) = charging_power_kw × facteur_CV
    // -------------------------------------------------------
    reflex do_charge when: (state = "charging") {
        // Calcul de l'énergie rechargée en kWh pendant ce cycle (step secondes)
        float hours_per_cycle <- step / 3600.0;   // Conversion secondes → heures
        float energy_kw;

        if (battery_level < 80.0) {
            // Phase CC : puissance nominale complète
            energy_kw <- charging_power_kw;
        } else {
            // Phase CV : puissance diminue linéairement de 100% → 10% entre 80%→100% batterie
            float soc_factor <- 1.0 - ((battery_level - 80.0) / 20.0) * 0.9;
            energy_kw <- charging_power_kw * max(0.1, soc_factor);
        }

        float energy_charged_kwh <- energy_kw * hours_per_cycle;
        float delta_soc <- (energy_charged_kwh / battery_capacity_kwh) * 100.0;

        // SoH : la batterie ne peut plus dépasser son SoH actuel
        float effective_max <- battery_soh;
        battery_level <- min(effective_max, battery_level + delta_soc);

        if (battery_level >= effective_max * 0.95) {
            ask target_station { do release_vehicle(myself); }
            target_station <- nil;
            state          <- "driving";
        }
    }

    // -------------------------------------------------------
    // FSM — ÉTAT : broken
    //   Récupération automatique après breakdown_recovery_cycles cycles.
    //   Simule un service de dépannage (recharge d'urgence à 25%).
    // -------------------------------------------------------
    reflex do_recovery when: (state = "broken") {
        if (breakdown_cycle = 0) { breakdown_cycle <- cycle; }

        // Clignotement visuel : alterner blink_visible toutes les 5 cycles
        // AMÉLIORATION 4 : Clignotement via cycle mod
        blink_visible <- (cycle mod 10) < 5;

        // Récupération après N cycles (dépannage)
        if (cycle - breakdown_cycle >= breakdown_recovery_cycles) {
            battery_level   <- 25.0;    // Recharge d'urgence à 25%
            state           <- "searching";
            breakdown_cycle <- 0;
            blink_visible   <- true;
            total_recoveries <- total_recoveries + 1;
        }
    }

    // -------------------------------------------------------
    // ACTION : Consommation d'énergie
    //   ALGORITHME 8 : Modèle physique kWh/km
    //   Formule : ΔE = consommation(kWh/km) × distance(km)
    //   La distance est calculée depuis la vitesse et le pas de temps.
    //   Nettement plus réaliste que ΔE = rate × speed.
    // -------------------------------------------------------
    action consume_energy {
        float dist_m     <- speed * step;                     // distance parcourue ce cycle (m)
        float dist_km    <- dist_m / 1000.0;                 // conversion en km
        float energy_kwh <- energy_consumption_kwh * dist_km; // énergie consommée (kWh)
        float delta_soc  <- (energy_kwh / battery_capacity_kwh) * 100.0; // variation SoH (%)

        battery_level     <- max(0.0, battery_level - delta_soc);
        distance_traveled <- distance_traveled + dist_m;
        total_km          <- total_km + dist_km;
        total_distance_km <- total_distance_km + dist_km;

        // ALGORITHME 10 : Mise à jour du SoH tous les 100 km de déplacement individuel
        do update_soh;
    }

    // -------------------------------------------------------
    // ACTION : Mise à jour du State of Health (SoH)
    //   ALGORITHME 10 : Dégradation linéaire du SoH
    //   SoH(km) = 100 - (km / 1000) × soh_degradation_per_1000km
    //   Le SoH ne descend jamais en dessous de soh_min (70%).
    // -------------------------------------------------------
    action update_soh {
        float new_soh <- max(soh_min, 100.0 - (total_km / 1000.0) * soh_degradation_per_1000km);
        battery_soh   <- new_soh;
        // Si la batterie dépasse le SoH courant (après dégradation), l'ajuster
        if (battery_level > battery_soh) {
            battery_level <- battery_soh;
        }
    }

    // -------------------------------------------------------
    // ACTION : Passage en panne
    // -------------------------------------------------------
    action breakdown {
        state            <- "broken";
        total_breakdowns <- total_breakdowns + 1;
        breakdown_cycle  <- 0;   // Sera initialisé au premier cycle dans do_recovery
    }

    // -------------------------------------------------------
    // ALGORITHME 3 : Sélection de station — Weighted Sum Method (WSM)
    //   Score = α × dist_normalisée + β × file_normalisée
    //   où α + β = 1, et α/β s'adaptent dynamiquement au niveau de batterie.
    //
    // ALGORITHME 4 : Adaptation dynamique des poids
    //   ratio = battery_level / battery_threshold
    //   Plus la batterie est faible (ratio → 0), plus α augmente
    //   (priorité à la station la PLUS PROCHE plutôt qu'à celle avec la plus courte file).
    //   C'est une heuristique de "Range Anxiety" inspirée des modèles comportementaux.
    // -------------------------------------------------------
    charging_station select_best_station {
        list<charging_station> candidates <- list(charging_station);
        if (empty(candidates)) { return nil; }

        // ALGORITHME 4 — Ratio dynamique
        float ratio <- (battery_threshold > 0.0) ? (battery_level / battery_threshold) : 1.0;
        float alpha <- alpha_base * min(1.0, ratio);
        float beta  <- 1.0 - alpha;

        float max_dist  <- max(candidates collect (self distance_to each.location));
        int   max_queue <- max(candidates collect (length(each.queue)));
        if (max_dist  <= 0.0) { max_dist  <- 1.0; }
        if (max_queue <= 0  ) { max_queue <- 1; }

        charging_station best       <- nil;
        float            best_score <- #infinity;

        // ALGORITHME 3 — WSM : minimiser le score composite
        loop s over: candidates {
            float nd <- (self distance_to s.location) / max_dist;
            float nq <- float(length(s.queue)) / float(max_queue);
            float sc <- alpha * nd + beta * nq;
            // Pénalité si la station est surchargée (file > capacité)
            if (s.is_full() and length(s.queue) > station_capacity) { sc <- sc + 0.5; }
            if (sc < best_score) { best_score <- sc; best <- s; }
        }
        return best;
    }

    // -------------------------------------------------------
    // VISUALISATION — Icône selon état + clignotement panne
    //   AMÉLIORATION 4 : Clignotement par alternance blink_visible
    //   géré dans le reflex do_recovery (cycle mod 10 < 5)
    // -------------------------------------------------------
    aspect icon_aspect {
        float icon_size <- 20.0;

        if (use_icons) {
            string img_path <- nil;

            if (state = "driving") {
                img_path <- "../images/car_blue.png";
            } else if (state = "searching") {
                img_path <- "../images/car_orange.png";
            } else if (state = "queuing") {
                img_path <- "../images/car_orange.png";
            } else if (state = "charging") {
                img_path <- "../images/car_green.png";
            } else {
                // broken — clignotement : n'afficher que si blink_visible = true
                if (blink_visible) {
                    img_path <- "../images/car_red.png";
                }
                // sinon img_path reste nil → rien n'est dessiné (effet clignotant)
            }

            if (img_path != nil) {
                draw image_file(img_path) size: {icon_size, icon_size};
            }

        } else {
            // Fallback formes géométriques avec clignotement par opacité
            rgb c;
            int alpha_val <- 255;
            if      (state = "driving")   { c <- #dodgerblue; }
            else if (state = "searching") { c <- #orange; }
            else if (state = "queuing")   { c <- #darkorange; }
            else if (state = "charging")  { c <- #limegreen; }
            else {
                c <- #red;
                alpha_val <- blink_visible ? 255 : 30;  // presque invisible quand blink_visible=false
            }
            draw circle(6) color: rgb(c.red, c.green, c.blue, alpha_val) border: #black;
        }

        // Barre de batterie (0→100% avec gradient vert→rouge)
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

        // Afficher le SoH si dégradé (< 95%)
        if (battery_soh < 95.0) {
            draw "SoH:" + string(int(battery_soh)) + "%"
                 at: {location.x + icon_size * 0.5, location.y + icon_size * 0.3}
                 color: #yellow font: font("Arial", 7, #plain);
        }
    }

    // Fallback sans images
    aspect base {
        rgb c;
        int alpha_val <- 255;
        if      (state = "driving")   { c <- #dodgerblue; }
        else if (state = "searching") { c <- #orange; }
        else if (state = "queuing")   { c <- #darkorange; }
        else if (state = "charging")  { c <- #limegreen; }
        else {
            c <- #red;
            alpha_val <- blink_visible ? 255 : 30;
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

    parameter "Nb véhicules"              var: nb_vehicles           min: 1      max: 500    category: "Simulation";
    parameter "Nb stations"               var: nb_stations           min: 1      max: 100    category: "Simulation";
    parameter "Capacité station"          var: station_capacity      min: 1      max: 20     category: "Simulation";
    parameter "Pas de temps (s/cycle)"    var: step                  min: 5.0    max: 120.0  category: "Simulation";

    parameter "Batterie initiale min (%)" var: initial_battery_min   min: 10.0   max: 100.0  category: "Batterie";
    parameter "Batterie initiale max (%)" var: initial_battery_max   min: 10.0   max: 100.0  category: "Batterie";
    parameter "Seuil d'alerte (%)"        var: battery_threshold     min: 5.0    max: 50.0   category: "Batterie";
    parameter "Capacité batterie (kWh)"   var: battery_capacity_kwh  min: 20.0   max: 150.0  category: "Batterie";
    parameter "Consommation (kWh/km)"     var: energy_consumption_kwh min: 0.10  max: 0.35   category: "Batterie";
    parameter "Puissance recharge (kW)"   var: charging_power_kw     min: 3.7    max: 150.0  category: "Batterie";
    parameter "Dégradation SoH (%/1000km)" var: soh_degradation_per_1000km min: 0.0 max: 2.0 category: "Batterie";

    parameter "Vitesse (km/h)"            var: vehicle_speed         min: 10.0   max: 130.0  category: "Déplacement";

    parameter "Alpha (poids distance)"    var: alpha_base            min: 0.0    max: 1.0    category: "Stratégie";
    parameter "Beta (poids file)"         var: beta_base             min: 0.0    max: 1.0    category: "Stratégie";
    parameter "Délai récupération panne"  var: breakdown_recovery_cycles min: 10 max: 500    category: "Stratégie";

    parameter "Utiliser les icônes"       var: use_icons                                     category: "Affichage";

    output {

        // Vue cartographique principale
        display "Carte GIS" type: opengl background: rgb(20, 20, 30) {
            species road             aspect: base;
            species charging_station aspect: icon_aspect;
            species vehicle          aspect: icon_aspect;
        }

        // Graphiques de monitoring
        display "Indicateurs" refresh: every(10 #cycles) {

            chart "États des véhicules" type: pie background: #white {
                data "Roulant"    value: vehicle count (each.state = "driving")   color: #dodgerblue;
                data "Recherche"  value: vehicle count (each.state = "searching") color: #orange;
                data "En file"    value: vehicle count (each.state = "queuing")   color: #darkorange;
                data "En charge"  value: vehicle count (each.state = "charging")  color: #limegreen;
                data "En panne"   value: vehicle count (each.state = "broken")    color: #red;
            }

            chart "Batterie moyenne (%)" type: series background: #white {
                data "Batterie moy."  value: mean(vehicle collect (each.battery_level))
                     color: #green style: line;
                data "SoH moyen (%)"  value: mean(vehicle collect (each.battery_soh))
                     color: #blue style: line;
                data "Seuil alerte"   value: battery_threshold
                     color: #red style: line;
            }

            chart "Occupation des stations (%)" type: histogram background: #white {
                loop s over: list(charging_station) {
                    data "St-" + int(s) value: s.occupancy_rate() * 100 color: #steelblue;
                }
            }

            chart "Pannes et recharges" type: series background: #white {
                data "Pannes"      value: total_breakdowns  color: #red   style: line;
                data "Recharges"   value: total_charges     color: #green style: line;
                data "Récupérations" value: total_recoveries color: #orange style: line;
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

        // Moniteurs
        monitor "Cycle"                 value: cycle;
        monitor "Pas simulé (s)"        value: step;
        monitor "Roulant"               value: vehicle count (each.state = "driving");
        monitor "En charge"             value: vehicle count (each.state = "charging");
        monitor "En file"               value: vehicle count (each.state = "queuing");
        monitor "En recherche"          value: vehicle count (each.state = "searching");
        monitor "En panne (actuel)"     value: vehicle count (each.state = "broken");
        monitor "Pannes totales"        value: total_breakdowns;
        monitor "Recharges totales"     value: total_charges;
        monitor "Récupérations"         value: total_recoveries;
        monitor "Distance totale (km)"  value: with_precision(total_distance_km, 1);
        monitor "Taux occ. moy. (%)"    value: with_precision(
                                            mean(charging_station collect (each.occupancy_rate())) * 100, 1);
        monitor "Batterie moy. (%)"     value: with_precision(
                                            mean(vehicle collect (each.battery_level)), 1);
        monitor "SoH moyen (%)"         value: with_precision(
                                            mean(vehicle collect (each.battery_soh)), 1);
        monitor "Temps attente total"   value: with_precision(total_wait_time, 1);
        monitor "Autonomie théo. (km)"  value: with_precision(
                                            battery_capacity_kwh / energy_consumption_kwh, 0);
    }
}

// =============================================================
//  EXPÉRIMENTATION BATCH
//  ALGORITHME 11 : Grid Search — Exploration exhaustive des combinaisons
//  de paramètres pour identifier les configurations optimales.
//  Chaque combinaison est répétée 3 fois (seeds différents) pour
//  la robustesse statistique des résultats.
// =============================================================
experiment BatchExploration type: batch repeat: 3 until: (cycle >= 5000) keep_seed: false {

    parameter "nb_vehicles"           var: nb_vehicles           among: [20, 50, 100, 200];
    parameter "nb_stations"           var: nb_stations           among: [5, 10, 15, 20];
    parameter "battery_threshold"     var: battery_threshold     among: [15.0, 25.0, 35.0];
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
