
## ----------------------------------Usesful Alias--------------------------------------
## -----------For tmux----------------
alias t='tmux'
alias tls='tmux ls'
alias tn='tmux new -s'
alias ta='tmux a -t'

## -----------For Python--------------
alias pl='pip list | grep '
alias cl='conda list | grep '
alias pi='pip install '
alias ci='conda install '

## alias for edit file quickly
alias sbrc="source ~/.bashrc"
alias vbrc="vim ~/.bashrc"

## for deep learning
alias g='gpustat -i'
alias n="nvidia-smi"
alias wn="watch -n 0.5 nvidia-smi"

# for gcc and g++
export CC=`which gcc`
export CXX=`which g++`

## For Slurm
alias s='squeue | grep haiming'

## alias for execute command quickly
alias aptu="sudo apt-get update"
alias aptg="sudo apt-get upgrade"
alias apti="sudo apt-get install"
alias r='reset'
alias p='pwd'
alias 'o'='nautilus'
alias opwd='nautilus $(pwd)'
alias so="source"
alias lsdn='ls -l|grep "^d"| wc -l' # count folder number
alias lsfn='ls -l|grep "^-"| wc -l'  # count files number
alias lsd='ls -d */' # only list directory in current path
alias df='df -hl'

## alias for switch path quickly
alias cdd='cd /data/data0/zhanghm' ## Change this path if neccessary

## -----------------Some other initialization-----------------
# export CUDA_DEVICE_ORDER=PCI_BUS_ID
# conda activate cv