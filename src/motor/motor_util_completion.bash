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
            -c|--check-messages-version) subcommand=check_messages_version ; break ;;
            position_tuning|current_tuning|stepper_tuning) subcommand=tuning ; break ;;
            stepper_velocity) subcommand=stepper_velocity ; break ;;
            voltage) subcommand=voltage ; break ;;
            state) subcommand=state ; break ;;
            tuning) subcommand=tuning_mode ; break ;;
        esac
        (( i-- ))
    done

    COMREPLY=()
    local words
    local base_words="-l --list -c --check-messages-version --no-list --list-names-only --list-path-only --list-devpath-only --list-serial-number-only --list-devnum-only --no-dfu-list -n --names -i --ips -a --uart-paths --uart-raw -p --paths -d --devpaths -s --serial_numbers set read --set-api --api --api-timing --run-stats --set-timeout -v --version -u --user-space --allow-simulated --lock -h --help";
    case $subcommand in
        set) words="--host_time --mode --current --position --velocity --torque --torque_dot --reserved --gpio state position_tuning current_tuning stepper_tuning voltage stepper_velocity tuning read -h --help";
            case $last in
                --host_time|--current|--position|--velocity|--reserved|--gpio) return 0 ;;
                --mode) words="open damped current position velocity torque impedance state current_tuning position_tuning voltage phase_lock stepper_tuning hardware_brake joint_position admittance find_limits driver_enable driver_disable clear_faults fault sleep crash reset" ;;
            esac ;;
        read) words="--poll --ppoll --aread --nonblock --frequency --statistics --read-write-statistics --text --fast_log -s --timestamp-in-seconds -t --host-time-seconds --publish --csv -f -r --reconnect --bits -v --compute-velocity --timestamp_frequency -p --precision -m --short set -h --help";
            case $last in
                --frequency|--timestamp_frequency|-p|--precision) return 0 ;;
            esac ;;
        names) words="$(motor_util --list-names-only) $base_words" ;;
        paths) words="$(motor_util --list-path-only) $base_words" ;;
        devpaths) words="$(motor_util --list-devpath-only) $base_words" ;;
        serial_numbers) words="$(motor_util --list-serial-number-only) $base_words" ;;
        check_messages_version) words="none major minor $base_words" ;;
        set_api) return 0 ;;
        state) words="--position --velocity --torque --torque_dot --kp --kd --kt --ks -h --help" ;
            case $last in
                --position|--velocity|--torque|--torque_dot|--kp|--kd|--kt|--ks) return 0 ;;
            esac ;;
        voltage) words="--voltage --velocity read -h --help" ;;
        stepper_velocity) words="--voltage --velocity --current --stepper_mode read -h --help" ;
            case $last in
                --voltage|--velocity|--current) return 0 ;;
                --stepper_mode) words="current voltage" ;;
            esac ;;
        tuning) words="--amplitude --frequency --mode --bias --kv read -h --help";
            case $last in
                --amplitude|--frequency|--bias) return 0 ;;
                --mode) words="sine square triangle chirp" ;;
            esac ;;
        tuning_mode) words="--amplitude --frequency --mode --bias --tuning_mode read -h --help";
            case $last in
                --amplitude|--frequency|--bias) return 0 ;;
                --tuning_mode) words="sine square triangle chirp" ;;
                --mode) words="position velocity torque" ;;
            esac ;;
        *) words=$base_words ;;
    esac

    COMPREPLY=($(compgen -W "$words" -- $cur))
}

complete -F _motor_util_completion motor_util