{
  pkgs,
  lib,
  config,
  inputs,
  ...
}:
let
in
# pkgs_stable = import inputs.stable_latest_nixpkgs { system = pkgs.stdenv.system; };
# pkgs_unstable = import inputs.unstable_nixkpgs { system = pkgs.stdenv.system; };
# nixos_2025_01_22 = import inputs.nixos_2025_01_22 { system = pkgs.stdenv.system; };
# nixpkgs_2025_01_22 = import inputs.nixpkgs_2025_01_22 { system = pkgs.stdenv.system; };
{
  cachix.enable = true;
  cachix.pull = [ "pre-commit-hooks" ];
  # Environmental variables
  env.GREET = "Welcome to 2D geometry constraint solver development environment!";

  # packages
  packages = [
    ## Build tools
    pkgs.gcc
    pkgs.gnumake
    pkgs.ninja
    # This is built from another commit because the one on nixpkgs is faulty for flakes
    # Prefer nixpkgs to keep it usable on other platforms as well
    pkgs.cmake

    ## Tools
    pkgs.doxygen
    pkgs.valgrind
    pkgs.virtualglLib

    ## Build deps
    pkgs.imgui
    pkgs.glfw
    pkgs.pkg-config
    pkgs.imnodes
  ];

  languages.python = {
    enable = true;
    version = "3.13";
    venv = {
      enable = true;
      requirements = '''';
    };
  };

  # Scripts
  enterShell = ''
    echo $GREET
  '';
}
