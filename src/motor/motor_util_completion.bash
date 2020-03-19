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
            read) subcommand=read ; break ;;
            --set_api) subcommand=set_api ; break ;;
            -n|--names) subcommand=names ; break ;;
        esac
        (( i-- ))
    done

    COMREPLY=()
    local words
    local base_words="-l --list --no-list --list-names-only --list-path-only -n --names set read --set_api --api -v --version -u --user-space -h --help";
    case $subcommand in
        set) words="--host_time --mode --current --position --velocity --reserved read -h --help";
            case $last in
                --host_time|--current|--position|--velocity|--reserved) return 0 ;;
                --mode) words="open damped current position velocity current current_tuning position_tuning voltage reset" ;;
            esac ;;
        read) words="--poll --aread --frequency --statistics --text -s --timestamp-in-seconds set -h --help";
            case $last in
                --frequency) return 0 ;;
            esac ;;
        names) words="$(motor_util --list-names-only) $base_words" ;;
        set_api) return 0 ;;
        *) words=$base_words ;;
    esac

    COMPREPLY=($(compgen -W "$words" -- $cur))
}

complete -F _motor_util_completion motor_util