(define (domain imv)

(:requirements :strips :typing)

(:types
  zone artifact robot pod - object

  cryo_zone - zone

  artifact_alpha artifact_beta - artifact

  capacity_score - object
)

; The zones will always be the same
; The pods will always be two
(:constants
  hall_alpha hall_beta maintenance_tunnel pod_zone entrance stasis_lab - zone
  cryo_chamber - cryo_zone

  pod1 pod2 - pod

  n0 n1 n2 n3 n4 - capacity_score
)

(:predicates

  ; Zone state
  (connected ?z1 - zone ?z2 - zone)
  (stable ?z - zone) ; Used to tell if a zone is stable
  (unstable ?z - zone)
  (low_pressure ?z - zone) ; If a zone has low pressure
  (normal_pressure ?z - zone)

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
  (is_sealed ?r - robot)
  (is_unsealed ?r - robot)
  (is_carrying ?r - robot ?a - artifact)
  (is_carrying_pod ?r - robot ?p - pod)
  (next_capacity ?n1 - capacity_score ?n2 - capacity_score)
  (robot_capacity ?r - robot ?n - capacity_score)
  (robot_can_carry ?r - robot ?n - capacity_score)
)

; Move to a normal pressure zone
(:action move_unsealed
  :parameters (?r - robot ?from - zone ?to - zone)
  :precondition (and 
    (connected ?from ?to)
    (robot_at ?r ?from)
    (stable ?to)
    (normal_pressure ?to)
    (is_unsealed ?r)
  )
  :effect (and 
    (not (robot_at ?r ?from))
    (robot_at ?r ?to)
  )
)

; Move to a high pressure zone
(:action move_sealed
  :parameters (?r - robot ?from - zone ?to - zone)
  :precondition (and 
    (connected ?from ?to)
    (robot_at ?r ?from)
    (stable ?to)
    (low_pressure ?to)
    (is_sealed ?r)
  )
  :effect (and 
    (not (robot_at ?r ?from))
    (robot_at ?r ?to)
  )
)

; Seal the robot
(:action seal
  :parameters (?r - robot)
  :precondition (is_unsealed ?r)
  :effect (and (is_sealed ?r) (not (is_unsealed ?r)))
)

; Unseal the robot
(:action unseal
  :parameters (?r - robot)
  :precondition (is_sealed ?r)
  :effect (and (not (is_sealed ?r)) (is_unsealed ?r))
)

; Load an artifact
(:action load
  :parameters (?r - robot ?a - artifact ?z - zone ?old_n - capacity_score ?new_n - capacity_score)
  :precondition (and 
    (robot_at ?r ?z)
    (artifact_at ?a ?z)
    (is_unsealed ?r)
    (robot_capacity ?r ?old_n)
    (next_capacity ?old_n ?new_n)
    (robot_can_carry ?r ?new_n)
  )
  :effect (and 
    (not (artifact_at ?a ?z))
    (is_carrying ?r ?a)
    (not (robot_capacity ?r ?old_n))
    (robot_capacity ?r ?new_n)
  )
)

; Unload an artifact
(:action unload
  :parameters (?r - robot ?a - artifact ?z - zone ?old_n - capacity_score ?new_n - capacity_score)
  :precondition (and 
    (robot_at ?r ?z)
    (is_carrying ?r ?a)
    (is_unsealed ?r)
    (robot_capacity ?r ?old_n)
    (next_capacity ?new_n ?old_n)

  )
  :effect (and 
    (artifact_at ?a ?z)
    (not (is_carrying ?r ?a))
    (not (robot_capacity ?r ?old_n))
    (robot_capacity ?r ?new_n)
  )
)

; Load a pod
(:action load_pod
  :parameters (?r - robot ?a - artifact ?p - pod ?z - zone ?old_n - capacity_score ?new_n - capacity_score)
  :precondition (and 
    (robot_at ?r ?z)
    (is_unsealed ?r)
    (pod_at ?p ?z)
    (in_pod ?a ?p)

    (robot_capacity ?r ?old_n)
    (next_capacity ?old_n ?new_n)
    (robot_can_carry ?r ?new_n)
  )
  :effect (and 
    (not (artifact_at ?a ?z))
    (not (pod_at ?p ?z))
    (is_carrying_pod ?r ?p)
    (not (robot_capacity ?r ?old_n))
    (robot_capacity ?r ?new_n)
  )
)

; Unload a pod
(:action unload_pod
  :parameters (?r - robot ?a - artifact ?p - pod ?z - zone ?old_n - capacity_score ?new_n - capacity_score)
  :precondition (and 
    (in_pod ?a ?p)
    (robot_at ?r ?z)
    (is_unsealed ?r)
    (is_carrying_pod ?r ?p)
    (robot_capacity ?r ?old_n)
    (next_capacity ?new_n ?old_n)
  )
  :effect (and 
    (pod_at ?p ?z)
    (not (is_carrying_pod ?r ?p))
    (not (robot_capacity ?r ?old_n))
    (robot_capacity ?r ?new_n)
  )
)

; after an artifact inside a pod is delivered succesfully a new pod is delivered in the pod zone
(:action reset_pod
  :parameters (?r - robot ?a - artifact_beta ?p - pod)
  :precondition (and 
    (robot_at ?r stasis_lab)
    (pod_at ?p stasis_lab)
    (in_pod ?a ?p)
  )
  :effect (and
    ; a new pod is delivered and its state is reset
    (artifact_at ?a stasis_lab)
    (not (pod_at ?p stasis_lab))
    (pod_at ?p pod_zone)
    (not (pod_full ?p))
    (pod_empty ?p)
    (not (in_pod ?a ?p))
  )
)

; cool down an artifact
(:action cool_artifact
  :parameters (?r - robot ?a - artifact_alpha ?z - cryo_zone)
  :precondition (and 
    (artifact_at ?a ?z)
    (robot_at ?r ?z)
    (is_unsealed ?r)
    (is_not_cool ?a)
  )
  :effect (and 
    (is_cool ?a)
    (not (is_not_cool ?a))
  )
)

; Place in pod an artifact
(:action place_in_pod
  :parameters (?r - robot ?a - artifact_beta ?z - zone ?p - pod)
  :precondition (and 
    (robot_at ?r ?z)
    (artifact_at ?a ?z)
    (is_unsealed ?r)
    (pod_at ?p ?z)
    (pod_empty ?p)
  )
  :effect (and 
    (not (pod_empty ?p))
    (pod_full ?p)
    (is_in_pod ?a)
    (in_pod ?a ?p)
    (not (artifact_at ?a ?z))
  )
)

; Wait the end of the seismic event
(:action end_seismic
  :parameters (?z - zone)
  :precondition (unstable ?z)
  :effect (and 
    (not (unstable ?z))
    (stable ?z)
  )
)

)