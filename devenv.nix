{
  pkgs,
  lib,
  config,
  inputs,
  ...
}:
let
  pkgs_unstable = import inputs.unstable_nixkpgs { system = pkgs.stdenv.system; };
  gcc15Stdenv = pkgs_unstable.gcc15Stdenv;
in
{
  cachix.enable = true;
  cachix.pull = [ "pre-commit-hooks" ];

  # Environmental variables
  env.GREET = "Welcome to 2D geometry constraint solver development environment!";
  env.CONAN_HOME = "${config.devenv.root}/.conan2";

  # packages
  packages = with pkgs_unstable; [
    ## Build tools
    gcc15Stdenv.cc
    gnumake
    ninja
    cmake
    clang-tools
    clang
    pkg-config

    ## Tools
    doxygen
    gdb
    valgrind

    ## GUI dependencies
    gtk4
    gtkmm4
    gsettings-desktop-schemas
    wrapGAppsHook4
    pkg-config

    ## Python
    (pkgs.python3.withPackages (
      python-pkgs: with python-pkgs; [
        conan
        mypy
      ]
    ))
    conan
  ];

  enterShell = ''
    export CC=clang
    export CXX=clang++

    # Make GTK4 GSettings schemas (e.g. FileChooser) discoverable
    # for development builds that are not wrapped by wrapGAppsHook.
    export XDG_DATA_DIRS="${pkgs_unstable.gtk4}/share/gsettings-schemas/${pkgs_unstable.gtk4.name}:$XDG_DATA_DIRS"

    echo ""
    echo "██████╗ ███╗   ███╗███████╗    ██╗██╗████████╗"
    echo "██╔══██╗████╗ ████║██╔════╝    ██║██║╚══██╔══╝"
    echo "██████╔╝██╔████╔██║█████╗      ██║██║   ██║   "
    echo "██╔══██╗██║╚██╔╝██║██╔══╝      ██║██║   ██║   "
    echo "██████╔╝██║ ╚═╝ ██║███████╗    ██║██║   ██║   "
    echo "╚═════╝ ╚═╝     ╚═╝╚══════╝    ╚═╝╚═╝   ╚═╝   "
    echo ""

    # Check git commit template
    echo "Checking whether git commit template is set for local project"
    if git config --get commit.template >/dev/null 2>&1; then
        echo "Commit template is already set, continuing"
    else
        TEMPLATE_CMD="git config commit.template .git_commit_msg_template"
        echo "No commit template configured, setting it for local project"
        echo "Executing: $TEMPLATE_CMD"
        $TEMPLATE_CMD
    fi
    echo

    # Check conan profile
    echo "Checking whether conan2 profile exists"
    if [ -d "$CONAN_HOME" ] && [ -n "$(ls -A "$CONAN_HOME" 2>/dev/null)" ]; then
        echo "Conan profile directory exists and contains files"
    else
        CONAN_CMD="conan profile detect --force"
        echo "Directory doesn't exist or is empty, creating conan profile"
        echo "Executing: $CONAN_CMD"
        $CONAN_CMD
    fi
    echo
  '';
}
