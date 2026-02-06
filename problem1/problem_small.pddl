(define (problem problem_small) (:domain imv)
(:objects 
  worker - robot

  alpha1 - artifact_alpha
  beta1 - artifact_beta
)

(:init
  ; Set the map composition
  (connected entrance maintenance_tunnel)
  (connected maintenance_tunnel entrance)
  (connected hall_alpha maintenance_tunnel)
  (connected maintenance_tunnel hall_alpha)
  (connected hall_beta maintenance_tunnel)
  (connected maintenance_tunnel hall_beta)
  (connected stasis_lab maintenance_tunnel)
  (connected maintenance_tunnel stasis_lab)
  (connected pod_zone maintenance_tunnel)
  (connected maintenance_tunnel pod_zone)
  (connected cryo_chamber maintenance_tunnel)
  (connected maintenance_tunnel cryo_chamber)

  (stable entrance)
  (stable maintenance_tunnel)
  (stable hall_alpha)
  (stable stasis_lab)
  (stable pod_zone)
  (stable cryo_chamber)
  ;(stable hall_beta)
  (unstable hall_beta)

  (normal_pressure entrance)
  (normal_pressure hall_alpha)
  (normal_pressure hall_beta)
  (normal_pressure stasis_lab)
  (normal_pressure cryo_chamber)
  (low_pressure maintenance_tunnel)
  (low_pressure pod_zone)

  (pod_at pod1 pod_zone)
  (pod_at pod2 pod_zone)
  (pod_empty pod1)
  (pod_empty pod2)

  (artifact_at alpha1 hall_alpha)
  (artifact_at beta1 hall_beta)
  (is_not_cool alpha1)

  (robot_at worker entrance)
  (is_unsealed worker)
  (robot_capacity worker n0)
  (robot_can_carry worker n0)
  (robot_can_carry worker n1)

  ; Sets how the capacity scores are ordered
  (next_capacity n0 n1)
  (next_capacity n1 n2)
  (next_capacity n2 n3)
  (next_capacity n3 n4)
)

(:goal (and
  (artifact_at alpha1 stasis_lab)
  (is_cool alpha1)
  (artifact_at beta1 stasis_lab)
  (is_in_pod beta1)
))

)
