#!/bin/bash

# Set the directory containing the .so files
LIB_DIR="."  # Change this to your directory path if needed

# Go through all .so files in the directory
find "$LIB_DIR" -maxdepth 1 -name "*.so*" -type f | while read -r SOFILE; do
  BASE_NAME=$(basename "$SOFILE")
  
  # Check if this is a versioned library like libfile.so.1.20.1
  if [[ "$BASE_NAME" =~ (.+)\.so\.([0-9]+)\.([0-9]+)\.([0-9]+) ]]; then
    LIB_NAME="${BASH_REMATCH[1]}"
    MAJOR="${BASH_REMATCH[2]}"
    
    # Create symlink libfile.so.1 -> libfile.so.1.20.1
    MINOR_LINK="${LIB_NAME}.so.${MAJOR}"
    if [ -L "$LIB_DIR/$MINOR_LINK" ]; then
      echo "Removing existing symlink: $MINOR_LINK"
      rm "$LIB_DIR/$MINOR_LINK"
    fi
    echo "Creating symlink: $MINOR_LINK -> $BASE_NAME"
    ln -sf "$BASE_NAME" "$LIB_DIR/$MINOR_LINK"
    
    # Create symlink libfile.so -> libfile.so.1
    MAIN_LINK="${LIB_NAME}.so"
    if [ -L "$LIB_DIR/$MAIN_LINK" ]; then
      echo "Removing existing symlink: $MAIN_LINK"
      rm "$LIB_DIR/$MAIN_LINK"
    fi
    echo "Creating symlink: $MAIN_LINK -> $MINOR_LINK"
    ln -sf "$MINOR_LINK" "$LIB_DIR/$MAIN_LINK"
  fi
  
  # Check if this is a versioned library like libfile.so.1
  if [[ "$BASE_NAME" =~ (.+)\.so\.([0-9]+)$ ]] && [ ! -L "$LIB_DIR/$BASE_NAME" ]; then
    LIB_NAME="${BASH_REMATCH[1]}"
    
    # Find the most recent version of this library
    FULL_VERSION=$(find "$LIB_DIR" -maxdepth 1 -name "${LIB_NAME}.so.${BASH_REMATCH[2]}.*" -type f | sort -V | tail -n 1)
    
    if [ -n "$FULL_VERSION" ]; then
      FULL_VERSION_NAME=$(basename "$FULL_VERSION")
      echo "Creating symlink: $BASE_NAME -> $FULL_VERSION_NAME"
      ln -sf "$FULL_VERSION_NAME" "$LIB_DIR/$BASE_NAME"
      
      # Create symlink libfile.so -> libfile.so.1
      MAIN_LINK="${LIB_NAME}.so"
      if [ -L "$LIB_DIR/$MAIN_LINK" ]; then
        echo "Removing existing symlink: $MAIN_LINK"
        rm "$LIB_DIR/$MAIN_LINK"
      fi
      echo "Creating symlink: $MAIN_LINK -> $BASE_NAME"
      ln -sf "$BASE_NAME" "$LIB_DIR/$MAIN_LINK"
    fi
  fi
done

rm -rf libglog.so.0

ln -sf libglog.so.1 libglog.so

ln -sf libglog.so.0.6.0 libglog.so.1

echo "Symlink repair completed."

