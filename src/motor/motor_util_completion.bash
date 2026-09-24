#!/usr/bin/env bash

_motor_util_completion()
{
    local cur="${COMP_WORDS[COMP_CWORD]}"

    # Pass the current cursor position and the entire command array to the executable
    # We use 2>/dev/null to ensure no spurious logs pollute the autocomplete dropdown
    local completions=$(motor_util --autocomplete ${COMP_CWORD} "${COMP_WORDS[@]}" 2>/dev/null)

    # compgen filters the completions based on the current word being typed
    COMPREPLY=( $(compgen -W "$completions" -- "$cur") )

    # Handle filename fallback for specific JSON flags if needed
    local last="${COMP_WORDS[$((${COMP_CWORD}-1))]}"
    case $last in
        -j|--json-ip-file|--json-mac-file)
            COMPREPLY=($(compgen -o plusdirs -f -X '!*.json' -- "$cur"))
            ;;
    esac
}

# -o filenames allows bash to handle path completions properly if the binary yields nothing
complete -o filenames -F _motor_util_completion motor_util
