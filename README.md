# MRS Core Examples

This repository includes Core ROS examples for the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system).

## Packages

## C++

* [example_waypoint_flier](./cpp/example_waypoint_flier) - Full C++ ROS2 component (nodelet) with "_MRS_" libraries and wrappers
* [example_controller_plugin](./cpp/example_controller_plugin) - Example of Controller plugin for the [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers)
* [example_tracker_plugin](./cpp/example_tracker_plugin) - Example of Tracker plugin for the [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers)
* [example_estimator_plugin](./cpp/example_estimator_plugin) - Example of Tracker plugin for the [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers)
* [example_pluginlib](./cpp/example_pluginlib) - Example of ROS Pluginlib, similar to how it is used in the [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers)
* [example_waypoint_flier_native](./cpp/example_waypoint_flier_native) - Minimalistic C++ ROS2 component (nodelet) with "_vanilla_" ROS features

## Python

* [example_sweeping_generator](./python/example_sweeping_generator) - Minimalistic Python Example that generates sweeping path for the UAV

## Tests

All the packages contain an example rostest.
You rostest to make sure your work is replicable and that your code can also work outside of your own installation.

# Disclaimer

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
