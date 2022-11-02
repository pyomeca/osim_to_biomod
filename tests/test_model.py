import os
from example.Models.model_evaluation import KinematicsTest, MomentArmTest
import numpy as np


def test_kinematics():
    parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    biomod_model = parent_path + "/example/Models/Wu_Shoulder_Model_via_points.bioMod"
    osim_model = parent_path + "/example/Models/Wu_Shoulder_Model_via_points.osim"
    kin_test = KinematicsTest(biomod=biomod_model, osim_model=osim_model)
    markers_error = kin_test.from_states(states=np.random.rand(16, 20) * 0.2, plot=False)
    np.testing.assert_almost_equal(np.mean(markers_error), 0, decimal=4)


def test_moment_arm():
    parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    np.random.seed(42)
    biomod_model = parent_path + "/example/Models/Wu_Shoulder_Model_via_points.bioMod"
    osim_model = parent_path + "/example/Models/Wu_Shoulder_Model_via_points.osim"
    muscle_test = MomentArmTest(biomod=biomod_model, osim_model=osim_model)
    muscle_error = muscle_test.from_markers(markers=np.random.rand(3, 22, 20), plot=False)
    np.testing.assert_almost_equal(np.mean(muscle_error), 0.0024544165693669525, decimal=4)
