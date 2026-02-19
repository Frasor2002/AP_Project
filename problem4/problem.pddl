(define (problem problem_4) (:domain imv)
(:objects 
  light1 heavy1 - ground_robot
  drone1 - drone

  alpha1 alpha2 - artifact_alpha
  beta1 beta2 - artifact_beta
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
  (unstable hall_beta)

  (normal_pressure entrance)
  (normal_pressure hall_alpha)
  (normal_pressure hall_beta)
  (normal_pressure stasis_lab)
  (normal_pressure cryo_chamber)
  (low_pressure maintenance_tunnel)
  (normal_pressure pod_zone)

  (pod_at pod1 pod_zone)
  (pod_at pod2 pod_zone)
  (pod_empty pod1)
  (pod_empty pod2)

  (artifact_at alpha1 hall_alpha)
  (artifact_at alpha2 hall_alpha)
  ;(artifact_at alpha3 hall_alpha)

  (artifact_at beta1 hall_beta)
  (artifact_at beta2 hall_beta)
  ;(artifact_at beta3 hall_beta)

  (is_not_cool alpha1)
  (is_not_cool alpha2)
  ;(is_not_cool alpha3)


  (robot_at light1 entrance)
  (robot_at heavy1 entrance)
  (robot_at drone1 entrance)

  (is_unsealed light1)
  (is_unsealed heavy1)

  (robot_capacity light1 n0)
  (robot_can_carry light1 n0)
  (robot_can_carry light1 n1)
  (robot_can_carry light1 n2)


  (robot_capacity heavy1 n0)
  (robot_can_carry heavy1 n0)
  (robot_can_carry heavy1 n1)
  (robot_can_carry heavy1 n2)
  (robot_can_carry heavy1 n3)
  (robot_can_carry heavy1 n4)

  
  (robot_capacity drone1 n0)
  (robot_can_carry drone1 n0)
  (robot_can_carry drone1 n1)

  ; Sets how the capacity scores are ordered
  (next_capacity n0 n1)
  (next_capacity n1 n2)
  (next_capacity n2 n3)
  (next_capacity n3 n4)
)

(:goal (and
  (artifact_at alpha1 stasis_lab)
  (artifact_at alpha2 stasis_lab)
  ;(artifact_at alpha3 stasis_lab)


  (is_cool alpha1)
  (is_cool alpha2)
  ;(is_cool alpha3)


  (artifact_at beta1 stasis_lab)
  (artifact_at beta2 stasis_lab)
  ;(artifact_at beta3 stasis_lab)


  (is_in_pod beta1)
  (is_in_pod beta2)
  ;(is_in_pod beta3)

))

(:metric minimize (total-time))

)
