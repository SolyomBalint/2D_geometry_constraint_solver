{
  pkgs,
  lib,
  config,
  inputs,
  ...
}:
let
  # pkgs_stable = import inputs.stable_latest_nixpkgs { system = pkgs.stdenv.system; };
  # pkgs_unstable = import inputs.unstable_nixkpgs { system = pkgs.stdenv.system; };
  nixos_2025_01_22 = import inputs.nixos_2025_01_22 { system = pkgs.stdenv.system; };
  nixpkgs_2025_01_22 = import inputs.nixpkgs_2025_01_22 { system = pkgs.stdenv.system; };
in
{
  # Environmental variables
  env.GREET = "Welcome to 2D geometry constraint solver development environment!";

  # packages
  packages = [
    nixpkgs_2025_01_22.gcc
    nixpkgs_2025_01_22.gnumake
    nixpkgs_2025_01_22.ninja

    # This is built from another commit because the one on nixpkgs is faulty for flakes
    # Prefer nixpkgs to keep it usable on other platforms as well
    nixos_2025_01_22.cmake

    pkgs.doxygen
    pkgs.valgrind
  ];

  languages.python.enable = true;
  languages.python.version = "3.13";
  languages.python.venv.enable = true;

  # Scripts
  enterShell = ''
    echo $GREET
  '';
}
