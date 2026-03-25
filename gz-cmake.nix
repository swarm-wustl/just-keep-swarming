{ lib, stdenv, pkgs, fetchFromGitHub, cmake, pkg-config }:

let
  # Use a Nix function to generate the config file content.
  # This avoids all shell quoting and substitution problems.
  gz-cmake4-config = pkgs.writeTextFile {
    name = "gz-cmake4-config.cmake";
    text = ''
      # --- Manually generated for Nix ---
      # 1. Add the module directory to CMake's search path.
      list(APPEND CMAKE_MODULE_PATH "''${CMAKE_CURRENT_LIST_DIR}/cmake5")

      # 2. Include the main file that defines the custom Gazebo functions.
      include(GzConfigure)
      # --- End of Nix generated file ---
    '';
  };
in
stdenv.mkDerivation rec {
  pname = "gz-cmake4";
  version = "4.2.0";

  src = fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-cmake";
    rev = "gz-cmake4_${version}";
    sha256 = "sha256-XF7oglj9Xr6F8a+6uowrY5a040yl4FZlFfW/Y0BJwOs=";
  };

  nativeBuildInputs = [ cmake pkg-config ];

  outputs = [ "out" "dev" ];

  # The postInstall hook is now much simpler.
  postInstall = ''
    mkdir -p $dev/share/cmake/gz-cmake4
    mv $out/share/cmake/gz-cmake/* $dev/share/cmake/gz-cmake4/
  
    # Overwrite the config with our custom one
    cp ${gz-cmake4-config} $dev/share/cmake/gz-cmake4/gz-cmake4-config.cmake
  '';

  meta = with lib; {
    description = "CMake modules for Gazebo (gz-cmake)";
    homepage = "https://gazebosim.org/";
    license = licenses.asl20;
    platforms = platforms.all;
  };
}
