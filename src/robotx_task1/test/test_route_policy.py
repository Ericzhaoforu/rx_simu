"""Arrival timing and route choice on crossing, stopped and lost traffic."""
import math
import unittest
import numpy as np
from robotx_task1.route_policy import RouteConfig, arrival_profile, assess_routes, route_metrics, remaining_path, route_conflict
from robotx_task1.traffic import Track


class RoutePolicyTests(unittest.TestCase):
    def setUp(self):
        self.hull=np.array([[-2.8,-1.27],[2.55,-1.27],[2.55,1.27],[-2.8,1.27]])
        self.direct=np.array([[0.,0.,0.],[20.,0.,0.]])
        self.detour=np.array([[0.,0.,0.],[0.,10.,math.pi/2],[20.,10.,0.],[20.,0.,-math.pi/2],[20.,0.,0.]])
        self.config=RouteConfig()

    def test_arrivals_include_acceleration_and_braking_and_are_monotonic(self):
        p=arrival_profile(self.direct,[0,0,0])
        self.assertGreater(p['eta'][-1],20/.3)
        self.assertLess(p['eta'][-1],20/.3+6.)
        self.assertTrue(np.all(np.diff(p['eta'])>0))
        self.assertTrue(np.all(p['earliest']<=p['eta']))
        self.assertTrue(np.all(p['latest']>=p['eta']))
        self.assertLessEqual(np.linalg.norm(np.diff(p['poses'][:,:2],axis=0),axis=1).max(),.400001)

    def test_heading_only_motion_has_finite_nonzero_time(self):
        p=arrival_profile(np.array([[0.,0.,0.],[0.,0.,math.pi/2]]),[0,0,0])
        self.assertAlmostEqual(p['eta'][-1],math.pi/2/.4)

    def test_crossing_has_a_finite_wait_that_changes_arrival_conflict(self):
        t=Track(1,10.,np.array([[8.,-5.],[10.,-2.]]),np.array([0.,.25]),True)
        result=route_metrics(self.direct,[0,0,0],self.hull,[t],10.,self.config)
        self.assertIsNotNone(result['conflict_eta_s'])
        self.assertGreater(result['wait_s'],0.)
        self.assertLessEqual(result['wait_s'],60.)

    def test_departed_traffic_does_not_force_a_wait(self):
        t=Track(1,10.,np.array([[8.,9.],[10.,12.]]),np.array([0.,.25]),True)
        result=route_metrics(self.direct,[0,0,0],self.hull,[t],10.,self.config)
        self.assertEqual(result['wait_s'],0.)

    def test_wait_checks_occupancy_before_departure_not_just_after_it(self):
        t=Track(1,10.,np.array([[-1.,-41.],[1.,-39.]]),np.array([0.,1.]),True)
        profile=arrival_profile(self.direct,[0,0,0])
        # Moving away immediately can be clear; sitting here for 60 seconds
        # would let the target cross the hull after about forty seconds.
        self.assertIsNone(route_conflict(profile,self.hull,[t],10.,0.,self.config))
        self.assertEqual(route_conflict(profile,self.hull,[t],10.,60.,self.config),0.)

    def test_waiting_for_crossing_beats_long_detour_with_switching_cost(self):
        t=Track(1,10.,np.array([[8.,-5.],[10.,-2.]]),np.array([0.,.25]),True)
        southern_detour=self.detour*np.array([1.,-1.,-1.])
        result,_=assess_routes(southern_detour,self.direct,[0,0,0],self.hull,[t],10.)
        self.assertTrue(result['prefer_wait'])
        self.assertGreater(result['switch_cost_s'],10.)
        self.assertLess(result['reference_score_s'],result['candidate_score_s'])

    def test_stationary_blockage_selects_available_detour_instead_of_endless_wait(self):
        t=Track(1,10.,np.array([[8.,-1.],[10.,1.]]),np.zeros(2),True)
        result,_=assess_routes(self.detour,self.direct,[0,0,0],self.hull,[t],10.)
        self.assertIsNone(result['reference_wait_s'])
        self.assertFalse(result['prefer_wait'])
        self.assertEqual(result['wait_s'],0.)

    def test_lost_track_does_not_disappear_from_arrival_evaluation(self):
        t=Track(1,10.,np.array([[8.,-1.],[10.,1.]]),np.array([0.,.25]),True)
        result=route_metrics(self.direct,[0,0,0],self.hull,[t],20.,self.config)
        self.assertIsNone(result['wait_s'])

    def test_different_goal_cannot_bias_a_new_stage(self):
        old=self.direct.copy();old[-1,0]=10.
        result,_=assess_routes(self.direct,old,[0,0,0],self.hull,[],10.)
        self.assertIsNone(result['reference_eta_s'])
        self.assertFalse(result['prefer_wait'])

    def test_small_refinement_does_not_pay_a_corridor_switch_penalty(self):
        refined=np.array([[0.,0.,0.],[10.,.2,0.],[20.,0.,0.]])
        result,_=assess_routes(refined,self.direct,[0,0,0],self.hull,[],10.)
        self.assertEqual(result['switch_cost_s'],0.)

    def test_distant_old_route_cannot_be_used_as_a_fictitious_shortcut(self):
        result,_=assess_routes(self.detour,self.direct,[0.,8.,math.pi/2],self.hull,[],10.)
        self.assertIsNone(result['reference_eta_s'])
        self.assertFalse(result['prefer_wait'])

    def test_loop_projection_uses_heading_to_choose_the_correct_leg(self):
        loop=np.array([[0.,0.,0.],[10.,0.,0.],[10.,1.,math.pi],[0.,.1,math.pi]])
        p=remaining_path(loop,[0.,0.,math.pi])
        self.assertLess(np.linalg.norm(p[-1,:2]-p[0,:2]),.2)
        self.assertLess(len(p),5)

    def test_invalid_configuration_is_rejected(self):
        for kw in ({'speed':0.},{'horizon':float('nan')},{'max_wait':-1.}):
            with self.assertRaises(ValueError):RouteConfig(**kw)


if __name__=='__main__':unittest.main()
