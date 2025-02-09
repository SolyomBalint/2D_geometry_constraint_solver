{
  pkgs,
  lib,
  config,
  inputs,
  ...
}:
let
  pkgs_devenv = import inputs.devenv_nixpkgs { system = pkgs.stdenv.system; };
  pkgs_stable = import inputs.stable_latest_nixpkgs { system = pkgs.stdenv.system; };
  pkgs_unstable = import inputs.unstable_nixkpgs { system = pkgs.stdenv.system; };
in
{
  # Environmental variables
  env.GREET = "Welcome to 2D geometry constraint solver development environment!";

  # packages
  packages = [ pkgs_unstable.cmake ];

  # Scripts
  enterShell = ''
    echo $GREET
  '';
}
