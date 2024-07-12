#!/bin/zsh
printError(){
    echo "ERROR: \e[31m${1}\e[0m"
}

printStep() {
    echo "STEP: \e[32m${1}\e[0m"
}

printInput(){
    echo "INPUT: \e[32m${1}\e[0m"
}
printInfo(){
    echo "INFO: \e[34m${1}\e[0m"
}
printText(){
    echo "${1}"
}
writeText(){
    echo "${1}" >> ${2}
}