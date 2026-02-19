(define (domain imv)

(:requirements :strips :typing :durative-actions :fluents)

(:types
  zone artifact robot pod - object
  artifact_alpha artifact_beta - artifact
  drone ground_robot - robot
)

; The zones will always be the same
; The pods will always be two


(:predicates

  ; Zone state
  (connected ?z1 - zone ?z2 - zone)
  (stable ?z - zone)
  (unstable ?z - zone)
  (low_pressure ?z - zone)
  (normal_pressure ?z - zone)
  (is_stasis ?z - zone)
  (is_pod_zone ?z - zone)
  (is_cryo ?z - zone)

  ; Pods state
  (pod_at ?p - pod ?z - zone)
  (pod_full ?p - pod)
  (pod_empty ?p - pod)
  (in_pod ?a - artifact_beta ?p - pod)

  ; Artifact state
  (artifact_at ?a - artifact ?z - zone)
  (is_cool ?a - artifact_alpha)
  (is_not_cool ?a - artifact_alpha)
  (is_in_pod ?a - artifact_beta)

  ; Robot state
  (robot_at ?r - robot ?z - zone)
  (is_sealed ?r - ground_robot)
  (is_unsealed ?r - ground_robot)
  (is_carrying ?r - robot ?a - artifact)
  (is_carrying_pod ?r - ground_robot ?p - pod)
)

(:functions
  (current_load ?r - robot)
  (max_capacity ?r - robot)
)

; drone
(:durative-action fly
  :parameters (?d - drone ?from - zone ?to - zone)
  :duration (= ?duration 2)
  :condition (and 
    (at start (robot_at ?d ?from))
    (over all (stable ?to))
    (over all (normal_pressure ?to))
  )
  :effect (and 
    (at start (not (robot_at ?d ?from)))
    (at end (robot_at ?d ?to))
  )
)

(:durative-action hook
  :parameters (?d - drone ?a - artifact ?z - zone)
  :duration (= ?duration 1)
  :condition (and 
    (at start (robot_at ?d ?z))
    (at start (artifact_at ?a ?z))
    (at start (< (current_load ?d) (max_capacity ?d)))
  )
  :effect (and 
    (at start (not (artifact_at ?a ?z)))
    (at end (is_carrying ?d ?a))
    (at start (increase (current_load ?d) 1))
  ))

(:durative-action unhook
  :parameters (?d - drone ?a - artifact ?z - zone)
  :duration (= ?duration 1)
  :condition (and 
    (at start (robot_at ?d ?z))
    (at start (is_carrying ?d ?a))
  )
  :effect (and 
    (at end (artifact_at ?a ?z))
    (at start (not (is_carrying ?d ?a)))
    (at start (decrease (current_load ?d) 1))
  )
)


; ground_robot

(:durative-action move_unsealed
  :parameters (?r - ground_robot ?from - zone ?to - zone)
  :duration (= ?duration 5)
  :condition (and 
    (at start (connected ?from ?to))
    (at start (robot_at ?r ?from))
    (over all (stable ?to))
    (over all (normal_pressure ?to))
    (at start (is_unsealed ?r))
  )
  :effect (and 
    (at start (not (robot_at ?r ?from)))
    (at end (robot_at ?r ?to))
  )
)

(:durative-action move_sealed
  :parameters (?r - ground_robot ?from - zone ?to - zone)
  :duration (= ?duration 5)
  :condition (and 
    (at start (connected ?from ?to))
    (at start (robot_at ?r ?from))
    (over all (stable ?to))
    (over all (low_pressure ?to))
    (at start  (is_sealed ?r))
  )
  :effect (and 
    (at start (not (robot_at ?r ?from)))
    (at end (robot_at ?r ?to))
  )
)

(:durative-action seal
  :parameters (?r - ground_robot)
  :duration (= ?duration 1)
  :condition (at start (is_unsealed ?r))
  :effect (and 
    (at end (is_sealed ?r)) 
    (at start (not (is_unsealed ?r)))
  )
)

(:durative-action unseal
  :parameters (?r - ground_robot)
  :duration (= ?duration 1)
  :condition (at start (is_sealed ?r))
  :effect (and 
    (at start (not (is_sealed ?r))) 
    (at end (is_unsealed ?r))
  )
)

(:durative-action load
  :parameters (?r - ground_robot ?a - artifact ?z - zone)
  :duration (= ?duration 3)
  :condition (and 
    (at start  (robot_at ?r ?z))
    (at start (artifact_at ?a ?z))
    (at start  (is_unsealed ?r))
    (at start (< (current_load ?r) (max_capacity ?r)))
  )
  :effect (and 
    (at start (not (artifact_at ?a ?z)))
    (at end (is_carrying ?r ?a))
    (at start (increase (current_load ?r) 1))
  )
)

(:durative-action unload
  :parameters (?r - ground_robot ?a - artifact ?z - zone)
  :duration (= ?duration 3)
  :condition (and 
    (at start (robot_at ?r ?z))
    (at start (is_carrying ?r ?a))
    (at start  (is_unsealed ?r))
  )
  :effect (and 
    (at end (artifact_at ?a ?z))
    (at start (not (is_carrying ?r ?a)))
    (at start (decrease (current_load ?r) 1))
  )
)

(:durative-action load_pod
  :parameters (?r - ground_robot ?a - artifact_beta ?p - pod ?z - zone)
  :duration (= ?duration 3)
  :condition (and 
    (at start  (robot_at ?r ?z))
    (at start  (is_unsealed ?r))
    (at start (pod_at ?p ?z))
    (at start (in_pod ?a ?p))
    (at start (< (current_load ?r) (max_capacity ?r)))
  )
  :effect (and 
    (at start (not (artifact_at ?a ?z)))
    (at start (not (pod_at ?p ?z)))
    (at end (is_carrying_pod ?r ?p))
    (at start (increase (current_load ?r) 1))
  )
)

(:durative-action unload_pod
  :parameters (?r - ground_robot ?a - artifact_beta ?p - pod ?z - zone)
  :duration (= ?duration 3)
  :condition (and 
    (at start  (in_pod ?a ?p))
    (at start (robot_at ?r ?z))
    (at start  (is_unsealed ?r))
    (at start (is_carrying_pod ?r ?p))
  )
  :effect (and 
    (at end (pod_at ?p ?z))
    (at start (not (is_carrying_pod ?r ?p)))
    (at start (decrease (current_load ?r) 1))
  )
)

; after an artifact inside a pod is delivered succesfully a new pod is delivered in the pod zone
(:durative-action reset_pod
  :parameters (?r - ground_robot ?a - artifact_beta ?p - pod ?sz - zone ?pz - zone)
  :duration (= ?duration 1)
  :condition (and 
    (at start (robot_at ?r ?sz))
    (at start (pod_at ?p ?sz))
    (at start (in_pod ?a ?p))
    (at start (is_stasis ?sz))
    (at start (is_pod_zone ?pz))
  )
  :effect (and
    ; a new pod is delivered and its state is reset
    (at end (artifact_at ?a ?sz))
    (at start (not (pod_at ?p ?sz)))
    (at end (pod_at ?p ?pz))
    (at start (not (pod_full ?p)))
    (at end (pod_empty ?p))
    (at start (not (in_pod ?a ?p)))
  )
)

(:durative-action cool_artifact
  :parameters (?r - ground_robot ?a - artifact_alpha ?cz - zone)
  :duration (= ?duration 10)
  :condition (and 
    (at start (is_cryo ?cz))
    (at start (artifact_at ?a ?cz))
    (at start (robot_at ?r ?cz))
    (at start (is_unsealed ?r))
    (at start (is_not_cool ?a))
  )
  :effect (and 
    (at end (is_cool ?a))
    (at start (not (is_not_cool ?a)))
  )
)

(:durative-action place_in_pod
  :parameters (?r - ground_robot ?a - artifact_beta ?z - zone ?p - pod)
  :duration (= ?duration 8)
  :condition (and 
    (at start (robot_at ?r ?z))
    (at start (artifact_at ?a ?z))
    (at start (is_unsealed ?r))
    (at start (pod_at ?p ?z))
    (at start (pod_empty ?p))
  )
  :effect (and 
    (at start (not (pod_empty ?p)))
    (at end (pod_full ?p))
    (at end (is_in_pod ?a))
    (at end (in_pod ?a ?p))
    (at start (not (artifact_at ?a ?z)))
  )
)

(:durative-action end_seismic
  :parameters (?z - zone)
  :duration (= ?duration 20)
  :condition (at start (unstable ?z))
  :effect (and 
    (at end (not (unstable ?z)))
    (at end (stable ?z))
  )
)
)