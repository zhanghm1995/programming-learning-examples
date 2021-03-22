# 该文件作为~/.zshrc文件的备份文件,用来记录一些重要的配置

plugins=(git z.lua extract)

source ~/.config/up/up.sh
# for autojump
#[[ -s ~/.autojump/etc/profile.d/autojump.sh ]] && source ~/.autojump/etc/profile.d/autojump.sh
eval "$(lua /home/zhanghm/.oh-my-zsh/custom/plugins/z.lua/z.lua  --init zsh)"

export PATH=/usr/local/cuda-10.0/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64:$LD_LIBRARY_PATH

source /opt/ros/kinetic/setup.zsh

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