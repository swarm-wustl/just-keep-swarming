{
  description = "Swarm";
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };
  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        rosDistro = "humble";

        pythonWithPackages = pkgs.python312.withPackages (p: with p; [
          numpy
          scipy
          opencv4
          debugpy
        ]);

        ignGazebo = pkgs.callPackage ./ign-gazebo.nix { };
      in {
        devShells.default = pkgs.mkShell {
          name = "Swarm";
          packages = with pkgs; [
            colcon
            opencv
            pythonWithPackages
            gz-cmake_3
            gz-utils_2
            ignGazebo

            (with rosPackages.${rosDistro}; buildEnv {
              paths = [
                ament-cmake
                ament-cmake-core
                ament-cmake-python
                python-cmake-module
                ros-core
                rclcpp
                rclpy
                rviz2
                cv-bridge
                joy
                joy-linux
                joy-teleop
                # gazebo
                # gazebo-dev
                # gazebo-model-attachment-plugin
                # gazebo-model-attachment-plugin-msgs
                # gazebo-msgs
                # gazebo-no-physics-plugin
                # gazebo-planar-move-plugin
                # gazebo-ros
                # gazebo-ros2-control
                # gazebo-set-joint-positions-plugin
                # gazebo-video-monitor-interfaces
                # gazebo-video-monitor-plugins
                # gazebo-video-monitor-utils
                # gazebo-video-monitors
              ];
            })
          ];
        };
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];

    permittedInsecurePackages = [
      "freeimage-3.18.0-unstable-2024-04-18"
    ];
  };
}
