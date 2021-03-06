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
            -p|--paths) subcommand=paths ; break ;;
            -d|--devpaths) subcommand=devpaths ; break ;;
            -s|--serial_numbers) subcommand=serial_numbers ; break ;;
            position_tuning|current_tuning|stepper_tuning) subcommand=tuning ; break ;;
            stepper_velocity|voltage) subcommand=voltage ; break ;;
        esac
        (( i-- ))
    done

    COMREPLY=()
    local words
    local base_words="-l --list -c --check-messages-version --no-list --list-names-only --list-path-only --list-devpath-only --list-serial-number-only -n --names -p --paths -d --devpaths -s --serial_numbers set read --set-api --api --run-stats -v --version -u --user-space --allow-simulated -h --help";
    case $subcommand in
        set) words="--host_time --mode --current --position --velocity --torque --reserved  position_tuning current_tuning stepper_tuning voltage stepper_velocity read -h --help";
            case $last in
                --host_time|--current|--position|--velocity|--reserved) return 0 ;;
                --mode) words="open damped current position velocity torque impedance current_tuning position_tuning voltage phase_lock stepper_tuning sleep crash reset" ;;
            esac ;;
        read) words="--poll --aread --frequency --statistics --read-write-statistics --text -s --timestamp-in-seconds -t --host-time-seconds --publish --csv -f --reserved-float -r --reconnect --bits set -h --help";
            case $last in
                --frequency) return 0 ;;
            esac ;;
        names) words="$(motor_util --list-names-only) $base_words" ;;
        paths) words="$(motor_util --list-path-only) $base_words" ;;
        devpaths) words="$(motor_util --list-devpath-only) $base_words" ;;
        serial_numbers) words="$(motor_util --list-serial-number-only) $base_words" ;;
        set_api) return 0 ;;
        voltage) words="--voltage --velocity read -h --help" ;;
        tuning) words="--amplitude --frequency --mode --bias --kv read -h --help";
            case $last in
                --amplitude|--frequency|--bias) return 0 ;;
                --mode) words="sine square triangle chirp" ;;
            esac ;;
        *) words=$base_words ;;
    esac

    COMPREPLY=($(compgen -W "$words" -- $cur))
}

complete -F _motor_util_completion motor_util