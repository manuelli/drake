# This awkward line exists to support zsh (instead of bash).
# Due to limitations of zsh, it must be the first line in the file. Sorry.
thisFile=$_
if [ $BASH ]
then
  # may be a relative or absolute path
  thisFile=${BASH_SOURCE[0]}
fi

set_drake_dirs()
{
  export DRAKE_SOURCE_DIR="$(cd "$(dirname "$thisFile")" && pwd)"
  export DRAKE_BUILD_DIR="$DRAKE_SOURCE_DIR/build"
}

set_paths()
{
  export PATH=$DRAKE_BUILD_DIR/install/bin:$DRAKE_BUILD_DIR/drake/bin:$PATH
  export PYTHONPATH=$DRAKE_BUILD_DIR/install/lib/python2.7/site-packages:$DRAKE_BUILD_DIR/install/lib/python2.7/dist-packages:$PYTHONPATH

  # add all jar files to java CLASSPATH
  for d in "$DRAKE_BUILD_DIR/install"; do
    if [ -d $d/share/java ]; then
      jd=$d/share/java
      for f in $jd/*.jar; do
        if [ -e $f ]; then
          CLASSPATH=$CLASSPATH:$f
        fi
      done
    fi
  done

  # extra special case to handle robotlocomotion-lcmtypes.jar
  export CLASSPATH=$CLASSPATH:$DRAKE_BUILD_DIR/install/share/java/robotlocomotion-lcmtypes.jar
}

set_drake_dirs
set_paths

# aliases
alias cddrake='cd "$DRAKE_SOURCE_DIR"'
alias makedrake='make -C "${DRAKE_BUILD_DIR}/drake"'
alias makedirector='make -C "${DRAKE_BUILD_DIR}/externals/director"'
