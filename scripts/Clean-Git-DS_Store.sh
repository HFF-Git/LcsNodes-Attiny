#!/bin/bash

# Clean-Git.sh
# Removes unwanted files/folders and updates .gitignore

# Add patterns to .gitignore if not already present
add_to_gitignore() {
  pattern=$1
  if ! grep -qx "$pattern" .gitignore 2>/dev/null; then
    echo "$pattern" >> .gitignore
    echo "Added $pattern to .gitignore"
  else
    echo "$pattern already in .gitignore"
  fi
}

# Add ignore rules
add_to_gitignore ".DS_Store"
add_to_gitignore "build/"
add_to_gitignore "bin/"

# Remove cached files so Git stops tracking them
git rm -r --cached .DS_Store 2>/dev/null
git rm -r --cached build 2>/dev/null
git rm -r --cached bin 2>/dev/null

# Stage the updated .gitignore
git add .gitignore

echo "Cleanup complete. Now run: git commit -m 'Clean repo and ignore unwanted files'"
