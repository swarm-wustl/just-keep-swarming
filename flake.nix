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
        pyPkgs = pkgs.python312Packages;
        rosDistro = "humble";
      in {
        devShells.default = pkgs.mkShell {
          name = "Swarm";
          packages = with pkgs; [
            colcon
            opencv
            pyPkgs.scipy
            pyPkgs.numpy
            pyPkgs.opencv4

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
              ];
            })
          ];
        };
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
