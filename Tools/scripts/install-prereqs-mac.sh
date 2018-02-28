#!/bin/bash
echo "Checking homebrew..."
$(which -s brew)
if [[ $? != 0 ]] ; then
    echo "installing homebrew..."
    ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
else
    echo "Homebrew installed"
fi

#install xconde dependencies
xcode-select --install

brew tap PX4/homebrew-px4
brew update
brew install genromfs
brew install gcc-arm-none-eabi
brew install gawk
brew install bash
brew install python

python easy_install pip  --user --force
pip install pyserial future catkin_pkg empy argparse pexpect lxml  --user --force
sudo pip install mavproxy
mavproxy.py --version
brew install gcc
brew link --overwrite gcc
sudo bash -c 'echo /usr/local/bin/bash >> /etc/shells'

ln -s /usr/local/bin/gcc-7 /usr/local/bin/gcc
ln -s /usr/local/bin/g++-7 /usr/local/bin/g++
ln -s /usr/local/bin/gcc-ranlib-7 /usr/local/bin/ranlib
ln -s /usr/local/bin/gcc-ar-7 /usr/local/bin/ar


SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
ARDUPILOT_ROOT=$(realpath "$SCRIPT_DIR/../../")
ARDUPILOT_TOOLS="Tools/autotest"

exportline="export PATH=$ARDUPILOT_ROOT/$ARDUPILOT_TOOLS:\$PATH";
grep -Fxq "$exportline" ~/.profile 2>/dev/null || {
    read -p "Add $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to your PATH [Y/n]?" -n 1 -r
    if [[ $REPLY =~ ^[Yy]$ ]] ; then
        echo $exportline >> ~/.profile
        eval $exportline
    else
        echo "Skipping adding $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to PATH."
    fi
}

git submodule update --init --recursive



if [[ $TRAVIS == 'true' ]]; then
    # ccache
    dir=$CCACHE_ROOT
    if [ ! -d "$HOME/opt/$dir" ]; then
        # if version 3 isn't there, try to remove older v2 folders from CI cache
        rm -rf "$HOME/opt"/ccache-3.2*

        wget https://www.samba.org/ftp/ccache/$CCACHE_TARBALL
        tar -xf $CCACHE_TARBALL
        pushd $CCACHE_ROOT
        ./configure --prefix="/tmp" --bindir="$HOME/opt/$dir"
        make
        make install
        popd
    fi
    mkdir -p $HOME/ccache

    # configure ccache
    ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/g++
    ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/gcc
    ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/arm-none-eabi-g++
    ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/arm-none-eabi-gcc
    ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/arm-linux-gnueabihf-g++
    ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/arm-linux-gnueabihf-gcc
    ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/clang++
    ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/clang
fi

. ~/.profile

echo "finished"
