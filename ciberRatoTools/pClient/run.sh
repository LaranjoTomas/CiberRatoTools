#!/bin/bash

challenge="1"
host="localhost"
robname="theAgent"
pos="0"
outfile="solution"

# Example of how to call the agent for each challenge
# ./run.sh -c 1 -h localhost -r theAgent -p 0 -f solution
# ./run.sh -c 2 -h localhost -r theAgent -p 0 -f solution
# ./run.sh -c 3 -h localhost -r theAgent -p 0 -f solution

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    1)
        # how to call agent for challenge 1
        python3 mainRob.py -h "$host" -p "$pos" -r "$robname"
        ;;
    2)
        # how to call agent for challenge 2
        python3 mainRob2.py -h "$host" -p "$pos" -r "$robname"
        ;;
    3)
        # how to call agent for challenge 3
        python3 mainRob3.py -h "$host" -p "$pos" -r "$robname"
        ;;
    4)
        # how to call agent for challenge 4
        python3 mainRob4.py -h "$host" -p "$pos" -r "$robname"
        ;;
esac