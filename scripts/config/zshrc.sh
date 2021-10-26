######################### 
#该文件作为~/.zshrc文件的配置文件备份,用来记录一些重要的配置
######################### 

## 常用oh-my-zsh插件
plugins=(git z.lua extract)

# 使用up来实现快速向上多级跳转
source ~/.config/up/up.sh

## 好用alias
alias t='nocorrect tmux'
alias vzrc="vim ~/.zshrc"
alias szrc="source ~/.zshrc"
alias sr="source devel/setup.zsh"
alias n="nvidia-smi"
alias wn="watch -n 0.5 nvidia-smi"
alias 'o'='nautilus'
alias 'opwd'='nautilus `pwd`'
alias so="source"
alias sbrc="source ~/.bashrc"
alias aptu="sudo apt-get update"
alias aptg="sudo apt-get upgrade"
alias apti="sudo apt-get install"
alias r="reset"
alias lsd='ls -d */' # only list directory in current path


# for autojump
#[[ -s ~/.autojump/etc/profile.d/autojump.sh ]] && source ~/.autojump/etc/profile.d/autojump.sh
eval "$(lua /home/zhanghm/.oh-my-zsh/custom/plugins/z.lua/z.lua  --init zsh)"

export PATH=/usr/local/cuda-10.0/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64:$LD_LIBRARY_PATH

# ROS
source /opt/ros/melodic/setup.zsh

# Python virtualenvs settings
export WORKON_HOME=$HOME/.virtualenvs
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
source /usr/local/bin/virtualenvwrapper.sh

# 清除全局历史,避免方向上下键找不到上次执行的命令
unsetopt inc_append_history
unsetopt share_history
unix-word-rubout() {
	  local WORDCHARS=$'!"#$%&\'()*+,-./:;<=>?@[\\]^_`{|}~'
	    zle backward-kill-word
}

zle -N unix-word-rubout
bindkey '^W' unix-word-rubout

eval $(thefuck --alias)
eval $(thefuck --alias f)

# 重定义rm函数,实现执行rm不永久删除,而是把删除的文件放在~/.Trash目录下,防止误删文件
function rm () {
  local path
  for path in "$@"; do
    # ignore any arguments
    if [[ "$path" = -* ]]; then :
    else
      local dst=${path##*/}
      # append the time if necessary
      while [ -e ~/.Trash/"$dst" ]; do
        dst="$dst "$(/bin/date +%H-%M-%S)
      done
      /bin/mv "$path" ~/.Trash/"$dst"
    fi
  done
}