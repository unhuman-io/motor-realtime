#!/usr/bin/env bash
_motor_util_completion()
{
    local cur=${COMP_WORDS[${COMP_CWORD}]}
    local last=${COMP_WORDS[$((${COMP_CWORD}-1))]}

    case $last in
        -h|--help) return 0;;
    esac

    local subcommand
    local i=$COMP_CWORD
    while [[ $i -gt 0 ]]
    do
        case ${COMP_WORDS[$i]} in
            set) subcommand=set ; break ;; 
        esac
        (( i-- ))
    done

    COMREPLY=()
    local words
    case $subcommand in
        set) words="--host_time --mode --current --position -h --help";
            case $last in
                --host_time|--mode|--current|--position) return 0;;
            esac ;;
        *) words="-l --list -p --print set -v --version -h --help";;
    esac

    COMPREPLY=($(compgen -W "$words" -- $cur))
}

complete -F _motor_util_completion motor_util