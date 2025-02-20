# MRS Core Examples

This repository includes Core ROS examples for the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system).

## Packages

## C++

* [waypoint_flier_simple](./cpp/waypoint_flier_simple) - Minimalistic C++ Nodelet with "_vanilla_" ROS features
* [waypoint_flier](./cpp/waypoint_flier) - Full C++ Nodelet with "_MRS_" libraries and wrappers
* [controller_plugin](./cpp/controller_plugin) - Example of Controller plugin for the [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers)
* [tracker_plugin](./cpp/tracker_plugin) - Example of Tracker plugin for the [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers)
* [pluginlib_example](./cpp/pluginlib_example) - Example of ROS Pluginlib, similar to how it is used in the [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers)

## Python

* [sweeping_generator](./python/sweeping_generator) - Minimalistic Python Example that generates sweeping path for the UAV
 
## tmux

* [passthrough_estimator](./tmux/passthrough_estimator) - tmux simulation session that shows how to run the simulation with passthrough estimator

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
