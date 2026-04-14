# Plan d'Implémentation — Améliorations Avancées EV Simulation

Ce document détaille le plan technique pour l'intégration des nouvelles fonctionnalités de la simulation (UI, Navigation réaliste, Modélisation physique, et Intelligence collective).

## User Review Required
> [!IMPORTANT]
> - **Système de Réservation** : Faut-il que la réservation bloque *strictement* la borne (personne ne peut s'y charger en attendant l'arrivée du véhicule ayant réservé), ou est-ce qu'on réserve simplement sa "place dans la file d'attente prioritaire" pour être certain de charger rapidement à l'arrivée (ce qui maximise le rendement de la station) ?
> - **Panne de station** : Doit-on introduire une probabilité de panne aléatoire pour les stations, ou souhaitez-vous un déclenchement manuel interactif depuis l'interface ? L'implémentation actuelle prévoit une probabilité aléatoire.

## Proposed Changes

### Composant : VÉHICULES (`vehicle`)

Modifications apportées à l'agent `vehicle`.

#### [MODIFY] ev_charging_simulation.gaml
- **Attributs ajoutés :** 
  - `float targetArrivalTime` (pour la réservation).
  - Gestion du chargement préventif.
- **Logique FSM (`driving` -> `searching`) :**
  - La transition à `searching` ne se base plus seulement sur le seuil statique `battery_threshold`.
  - Ajout d'une condition préventive : Si l'énergie nécessaire pour atteindre `target_point` + l'énergie pour rejoindre la station la plus proche > `battery_level`, le véhicule passe en recharge anticipée.
- **Réaction aux pannes de stations (`queuing`) :**
  - Si `target_station.isOperational` devient faux, le véhicule annule sa cible et repasse en mode `searching` pour trouver une alternative.
- **Priorité de Charge :**
  - Ajout de la méthode `int get_priority()` : `delivery` retourne 3, `taxi` retourne 2, `personal` retourne 1.
- **Aspect Visuel (IHM) :**
  - Ajout d'une étiquette numérique avec le pourcentage du SoC (State of Charge) (ex: `84%`) dessinée au-dessus du véhicule pour une clarté instantanée.

### Composant : STATIONS DE RECHARGE (`charging_station`)

Modifications apportées à l'agent `charging_station`.

#### [MODIFY] ev_charging_simulation.gaml
- **Nouvel Attribut :** `bool isOperational <- true;`
- **Nouvelle Méthode :** `action reserveSlot(vehicle v)`
  - Permet d'incrémenter un compteur `reserved_slots` lorsqu'un véhicule en mode `searching` la choisit.
- **Gestion des Pannes :**
  - Ajout de réflexes `breakdown` et `repair` permettant à la station de tomber en panne aléatoirement (avec changement visuel de couleur/icône, par exemple en gris) et de refuser toute affectation ou charge.
- **Logique de File Prioritaire (`add_to_queue`) :**
  - La file n'est plus strictement FIFO. Lorsqu'un véhicule s'ajoute, la file est triée selon sa priorité (les bus/camions doublent les particuliers).
- **Puissance de charge adaptative :**
  - Le calcul de `effective_power_kw` prend en compte le SoH (une batterie dégradée limite la puissance absorbable pour éviter la surchauffe).
  - La puissance max dépend aussi du type (`delivery` encaisse plus de kW que `personal`).

## Open Questions
- Quel est le comportement attendu si un véhicule a réservé une place mais prend du retard ou tombe en panne en route ? La réservation doit-elle avoir un "timeout" d'expiration ?
- Souhaitez-vous que les stations inopérantes soient affichées avec une icône visuelle particulière ? (Nous pouvons utiliser une teinte grisée ou une croix).

## Verification Plan
### Automated Tests
- La compilation du code GAML se fera via l'IDE GAMA sans erreurs.
### Manual Verification
- **Lancer la simulation et vérifier visuellement :**
  - Le pourcentage SoC est visible au-dessus des véhicules.
  - Les stations tombent parfois en panne (elles deviendront visuellement inactives) et les véhicules en attente se redirigent.
  - Dans l'état de la file (via inspection GAMA), vérifier que les camions (`delivery`) sont classés avant les particuliers (`personal`) dans l'attente `queue`.
