#!/usr/bin/env bash

if [ $# -le 1 ] || [ "$1" = "--help" ]; then
  echo "usage: ./repurpose_package.sh <original_package_name> <new_package_name> [--dry-run, --no-camel-case]
  Removes the .git directory, initializes a new one,
  replaces all occurences of the original package name with the new one in all files within the package,
  replaces all occurences of the original package name with the new one within all filenames."
  exit 1
fi

ORIG_NAME="$1"
NEW_NAME="$2"
DRY_RUN=0
CAMEL_CASE=1

if [ "$ORIG_NAME" = "$NEW_NAME" ]; then
  echo "Package names must be different."
  exit 1
fi

if [[ " ${*:3} " == *" --dry-run "* ]]; then
  echo "Dry run mode on - nothing will be changed."
  DRY_RUN=1
fi

if [[ " ${*:3} " == *" --no-camel-case "* ]]; then
  echo "CamelCase occurences will not be replaced."
  CAMEL_CASE=0
fi

# Try to find and then change into the given directory
dir=$(find . -type d -name "$ORIG_NAME" -print -quit)

if [ -n "$dir" ]; then
  cd "$dir" || exit 1
else
  echo "Package $ORIG_NAME not found."
  exit 1
fi

# Prepare variants of the package names without example_ and _plugin
ORIG_VARIANT=("$ORIG_NAME")
NEW_VARIANT=("$NEW_NAME")

for pattern in "#example_" "%_plugin"; do
  eval "ORIG_TEMP=\${ORIG_NAME$pattern}"
  eval "NEW_TEMP=\${NEW_NAME$pattern}"

  if [ "$ORIG_NAME" != "$ORIG_TEMP" ]; then
    ORIG_VARIANT=("$ORIG_TEMP" "${ORIG_VARIANT[@]}")
    NEW_VARIANT=("$NEW_TEMP" "${NEW_VARIANT[@]}")
  fi
done

# #{ init_new_repo()

init_new_repo() {
  local DRY_RUN="$1"

  # Remove .git and initialize a new repository
  echo "** CREATING NEW GIT REPOSITORY **"

  # Ask for confirmation
  echo History of the old repository will be deleted.
  read -p "Are you sure? (y/n) " -r

  if [[ "$REPLY" =~ ^[Yy]$ ]]; then
    if [ "$DRY_RUN" -eq 0 ]; then
      rm -rf .git
      git init
    fi

    echo Created a new git repository.
  else
    echo Not creating a new git repository.
  fi
}

# #}

# #{ replace_within_files ()

replace_within_files() {
  local ORIG_NAME="$1"
  local NEW_NAME="$2"
  local DRY_RUN="$3"

  # Replace occurences within files
  readarray -d '' within_files < <(grep -rl "$ORIG_NAME" --null)
  echo "** REPLACING OCCURENCES WITHIN FILES **"

  if [[ -z "${within_files[*]}" ]]; then
    echo "No matches found for \"$ORIG_NAME\" within files!"
  else
    # Ask for confirmation
    echo These files will be modified:

    for file in "${within_files[@]}"; do
      echo " - $file"
    done

    read -p "Are you sure? (y/n) " -r
    if [[ "$REPLY" =~ ^[Yy]$ ]]; then
      for file in "${within_files[@]}"; do
        echo " - Replacing \"$1\" with \"$2\" in file \"$file\"."

        if [ "$DRY_RUN" -eq 0 ]; then
          sed -i "s/$ORIG_NAME/$NEW_NAME/g" "$file"
        fi
      done

      echo "Replaced all occurences within files."
    else
      echo "Not replacing occurences within files."
    fi
  fi
}

# #}

# #{ replace_within_filenames()

replace_within_filenames() {
  local ORIG_NAME="$1"
  local NEW_NAME="$2"
  local DRY_RUN="$3"

  # Replace occurences in file names
  readarray -d '' files < <(find . -name "*$ORIG_NAME*" -print0)
  echo "** REPLACING OCCURENCES WITHIN FILE NAMES **"

  if [[ -z "${files[*]}" ]]; then
    echo "No matches found for \"$ORIG_NAME\" in file names!"
  else
    # Ask for confirmation
    echo These files will be modified:

    for file in "${files[@]}"; do
      echo " - $file"
    done

    read -p "Are you sure? (y/n) " -r
    if [[ "$REPLY" =~ ^[Yy]$ ]]; then
      for file in "${files[@]}"; do
        new_file=${file//$ORIG_NAME/$NEW_NAME}

        echo " - Renaming file \"$file\" to \"$new_file\"."
        if [ "$DRY_RUN" -eq 0 ]; then
          mv "$file" "$new_file"
        fi
      done

      echo "Replaced all occurences within file name."
    else
      echo "Not replacing occurences within file name."
    fi
  fi
}

# #}

init_new_repo "$DRY_RUN"
for i in "${!ORIG_VARIANT[@]}"; do
  replace_within_files "${ORIG_VARIANT[i]}" "${NEW_VARIANT[i]}" "$DRY_RUN"
  replace_within_filenames "${ORIG_VARIANT[i]}" "${NEW_VARIANT[i]}" "$DRY_RUN"

  if [ "$CAMEL_CASE" -eq 1 ]; then
    ORIG_CC=$(echo "${ORIG_VARIANT[i]}" | sed -r 's/(^|_)([a-z])/\U\2/g')
    NEW_CC=$(echo "${NEW_VARIANT[i]}" | sed -r 's/(^|_)([a-z])/\U\2/g')

    replace_within_files "$ORIG_CC" "$NEW_CC" "$DRY_RUN"
    replace_within_filenames "$ORIG_CC" "$NEW_CC" "$DRY_RUN"
  fi
done

if [ "$DRY_RUN" -eq 1 ]; then
  echo "Dry run mode on - nothing was changed."
fi
