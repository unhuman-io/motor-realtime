#!/bin/bash

cd $(dirname ${BASH_SOURCE[0]})

cat <<EOF > motor_util_tree.md
# motor_util help
\`\`\`
$(motor_util -h)
\`\`\`

## set subcommand
\`\`\`
$(motor_util set -h)
\`\`\`

### subcommands of set
\`\`\`
$(motor_util set voltage -h)
\`\`\`

\`\`\`
$(motor_util set current_tuning -h)
\`\`\`

\`\`\`
$(motor_util set position_tuning -h)
\`\`\`

\`\`\`
$(motor_util set stepper_tuning -h)
\`\`\`

## read subcommand
\`\`\`
$(motor_util read -h)
\`\`\`
EOF
