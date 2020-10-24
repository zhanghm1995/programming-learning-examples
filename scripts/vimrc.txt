# vim常用配置选项,放置在~/.vimrc下,没有扩展名

set mouse=a
set number
set showcmd
set ruler
" set laststatus=2
" set list

" 自动缩进
set autoindent
set smartindent
"""""""""" 代码配置:
set showmatch
" 打开语法高亮显示
syntax on
" filetype on
set cursorline
highlight CursorLine   cterm=NONE ctermbg=darkgray ctermfg=None guibg=NONE guifg=NONE

" 快捷键设置
" 设置Ctrl+A为全选用 nnoremap <C-A> ggVG,下面设置的VV为全选
nnoremap VV ggVG
noremap <C-c> "+y

set aw
