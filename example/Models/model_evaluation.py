import biorbd
from math import ceil
try:
    import bioviz
    bioviz_pack = True
except ModuleNotFoundError:
    bioviz_pack = False
import numpy as np
from osim_to_biomod.utils import *
import matplotlib.pyplot as plt

try:
    import opensim as osim
    import pyosim
    osim_pack = True
except ModuleNotFoundError:
    osim_pack = False


class KinematicsTest:
    def __init__(self, biomod, osim_model):
        self.biomod_model = biorbd.Model(biomod)
        self.osim_model = osim.Model(osim_model)
        self.marker_names = None
        self.markers = None

    def from_markers(self, markers: np.ndarray, marker_names: list = None, plot: bool = True, save: bool = True):
        """
        Run test using markers data:
        1) inverse kinematic using biorbd
        2) apply the states on both model
        3) compare the markers positions during the movement
        """
        if markers.shape[1] != self.osim_model.getMarkerSet().getSize():
            raise RuntimeError("The number of markers in the model and the markers data must be the same.")
        elif markers.shape[0] != 3:
            raise RuntimeError("The markers data must be a 3D array of dim (3, n_markers, n_frames).")

        # 1) inverse kinematic using biorbd
        states = self._run_inverse_kin(markers)
        self.marker_names = marker_names
        self.markers = markers
        return self.from_states(states=states, plot=plot, save=save)

    def from_states(self, states, plot: bool = True, save: bool = True) -> np.ndarray:
        nb_markers = self.osim_model.getMarkerSet().getSize()
        nb_frame = states.shape[1]
        osim_markers = np.ndarray((3, nb_markers, nb_frame))
        biorbd_markers = np.ndarray((3, nb_markers, nb_frame))
        markers_error = []
        ordered_osim_idx = self._reorder_osim_coordinate()
        osim_state = states.copy()
        my_state = self.osim_model.initSystem()
        for i in range(nb_frame):
            for b in range(states.shape[0]):
                if self.osim_model.getCoordinateSet().get(ordered_osim_idx[b]).getDefaultValue() != 0:
                    osim_state[b, i] = states[b, i] + self.osim_model.getCoordinateSet().get(ordered_osim_idx[b]).getDefaultValue()
                self.osim_model.getCoordinateSet().get(ordered_osim_idx[b]).setValue(my_state, osim_state[b, i])
            bio_markers_array = self.biomod_model.markers(states[:, i])
            osim_markers_names = [self.osim_model.getMarkerSet().get(m).toString() for m in range(self.osim_model.getMarkerSet().getSize())]
            osim_marker_idx = []
            for m in range(nb_markers):
                if self.marker_names and self.marker_names[m] != self.osim_model.getMarkerSet().get(m).getName():
                    raise RuntimeError(
                        "Markers names are not the same between names and opensim model. Place markers in teh same order as the model.")
                osim_idx = osim_markers_names.index(self.biomod_model.markerNames()[m].to_string())
                osim_marker_idx.append(osim_idx)
                osim_markers[:, m, i] = self.osim_model.getMarkerSet().get(osim_idx).getLocationInGround(my_state).to_numpy()
                biorbd_markers[:, m, i] = bio_markers_array[m].to_array()
                markers_error.append(np.mean(np.sqrt((osim_markers[:, m, i] - biorbd_markers[:, m, i])**2)))

        # 3) compare the markers positions during the movement
        if plot:
            default_nb_line = 5
            var = ceil(nb_markers/ default_nb_line)
            nb_line = var if var < default_nb_line else default_nb_line
            # plot osim marker and biomod markers in subplots
            plt.figure("Markers (titles : (osim/biorbd))")
            list_labels = ["osim markers", "biorbd markers"]
            for m in range(nb_markers):
                plt.subplot(nb_line, ceil(nb_markers/nb_line), m + 1)
                for i in range(3):
                    if self.markers:
                        plt.plot(self.markers[i, m, :], "r--")
                        list_labels = ["experimental markers"] + list_labels
                    plt.plot(osim_markers[i, m, :], "b")
                    plt.plot(biorbd_markers[i, m, :], "g")
                plt.title(f"{self.osim_model.getMarkerSet().get(osim_marker_idx[m]).getName()}/"
                          f"{self.biomod_model.markerNames()[m].to_string()}")
                if m == 0:
                    plt.legend(labels=list_labels)
                # axs[m].legend()
            plt.figure("states (titles : (osim/biorbd))")
            var = ceil(states.shape[0]/default_nb_line)
            nb_line = var if var < default_nb_line else default_nb_line
            for i in range(states.shape[0]):
                plt.subplot(nb_line, ceil(states.shape[0]/nb_line), i + 1)
                plt.plot(osim_state[i, :], "b")
                plt.plot(states[i, :], "g")
                plt.title(f"{self.osim_model.getCoordinateSet().get(ordered_osim_idx[i]).getName()}/"
                          f"{self.biomod_model.nameDof()[i].to_string()}")
                if i == 0:
                    plt.legend(labels=["osim states (handle default value)", "states"])
            plt.show()
        return markers_error

    # def from_states(self, states):
    def _reorder_osim_coordinate(self):
        """
        Reorder the coordinates to have translation after rotation like biorbd model
        """
        tot_idx = 0
        ordered_idx = []
        for i in range(self.osim_model.getJointSet().getSize()):
            translation_idx = []
            rotation_idx = []
            for j in range(self.osim_model.getJointSet().get(i).numCoordinates()):
                if not self.osim_model.getJointSet().get(i).get_coordinates(j).get_locked():
                    if self.osim_model.getJointSet().get(i).get_coordinates(j).getMotionType() == 1:
                        translation_idx.append(tot_idx + j)
                    elif self.osim_model.getJointSet().get(i).get_coordinates(j).getMotionType() == 3:
                        rotation_idx.append(tot_idx + j)
                    else:
                        raise RuntimeError("Unknown motionType.")
            tot_idx += self.osim_model.getJointSet().get(i).numCoordinates()
            ordered_idx += rotation_idx + translation_idx
        return ordered_idx

    def load_movement(self, states):
        self.states = states

    def _run_inverse_kin(self, markers: np.ndarray) -> np.ndarray:
        """
        Run biorbd inverse kinematics
        Parameters
        ----------
        markers: np.ndarray
            Markers data
        Returns
        -------
            states: np.ndarray
        """
        ik = biorbd.InverseKinematics(self.biomod_model, markers)
        ik.solve()
        return ik.q

class LeverArmTest:
    def __init__(self):
        pass


class VisualizeModel(bioviz.Viz):
    def __init__(self, model_path, **kwargs):
        super().__init__(model_path=model_path, **kwargs)


if __name__ == '__main__':
    #viz = VisualizeModel("Wu_Shoulder_Model_via_points.bioMod", )
    #viz.exec()
    kin_test = KinematicsTest(biomod="Wu_Shoulder_Model_via_points.bioMod", osim_model="Wu_Shoulder_Model_via_points.osim")
    kin_test.from_states(states=np.random.rand(16, 20)*0.2,  plot=True, save=True)
    #kin_test.from_states(states=np.zeros((2, 2)), plot=True, save=True)
    # viz = VisualizeModel("Wu_Shoulder_Model_via_points.bioMod", )
    # viz.exec()




