#!/usr/bin/env bash

if [ $# -le 1 ]; then
  echo "usage: ./repurpose_package.sh <original_package_name> <new_package_name> [--dry-run]"
  if [ $1 == "--help" ]; then
    echo "
    Removes the .git directory, initializes a new one,
    replaces all occurences of the original package name with the new one in all files withing all subfolders,
    replaces all occurences of the original package name with the new one wthinin all filenames."
  fi
  exit 1
fi

ORIG_NAME=$1
NEW_NAME=$2

if [ "$3" == "--dry-run" ]; then
  echo "Dry run mode on - nothing will be changed"
fi

# Remove .git and initialize a new repository

echo "** CREATING NEW GIT REPOSITORY **"

# Ask for confirmation
echo History of the old repository will be deleted.
read -p "Are you sure? " -r

if [[ $REPLY =~ ^[Yy]$ ]]
then

  if [ "$3" != "--dry-run" ]; then
    rm -rf .git
    git init
  fi

fi

# Replace occurences within files
readarray -d '' within_files < <(grep -rl "$ORIG_NAME" --null)

echo "** REPLACING OCCURENCES WITHIN FILES **"

if [[ -z "${within_files[@]}" ]]; then

  echo "No matches found for \"$1\" within files!"

else

  # Ask for confirmation
  echo These files will be modified:
  for file in "${within_files[@]}"
  do
    echo " - $file"
  done
  read -p "Are you sure? " -r

  if [[ $REPLY =~ ^[Yy]$ ]]
  then

    for file in "${within_files[@]}"
    do
      echo " - Replacing \"$1\" with \"$2\" in file \"$file\"."
      if [ "$3" != "--dry-run" ]; then
        sed -i "s/$ORIG_NAME/$NEW_NAME/g" "$file"
      fi
    done

  fi

fi

# Replace occurences in file names
readarray -d '' files < <(find -name "*$ORIG_NAME*" -print0)

echo "** REPLACING OCCURENCES WITHIN FILE NAMES **"

if [[ -z "${files[@]}" ]]; then

  echo "No matches found for \"$1\" in file names!"

else

  # Ask for confirmation
  echo These files will be modified:
  for file in "${files[@]}"
  do
    echo " - $file"
  done
  read -p "Are you sure? " -r

  if [[ $REPLY =~ ^[Yy]$ ]]
  then


  for file in "${files[@]}"
  do
    new_file=${file//$ORIG_NAME/$NEW_NAME}
    echo " - Renaming file \"$file\" to \"$new_file\"."
    if [ "$3" != "--dry-run" ]; then
      mv "$file" "$new_file"
    fi
  done

  fi

fi

if [ "$3" == "--dry-run" ]; then
  echo "Dry run mode on - nothing was changed"
fi

