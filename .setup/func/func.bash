#!/bin/bash
printError(){
    echo -e "ERROR: \e[31m${1}\e[0m"
}

printStep() {
    echo -e "STEP: \e[32m${1}\e[0m"
}

printInput(){
    echo -e "INPUT: \e[32m${1}\e[0m"
}

printInfo(){
    echo -e "INFO: \e[34m${1}\e[0m"
}

printText(){
    echo -e "${1}"
}
writeText(){
    echo -e "${1}" >> ${2}
}