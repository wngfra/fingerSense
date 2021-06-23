syntax enable
set mouse=a
set expandtab
set shiftwidth=4
set tabstop=4
set smartindent
set autoindent
set number relativenumber
set cindent
set noshowmode
filetype on

" Install vim-plug
if empty(glob('~/.vim/autoload/plug.vim'))
    silent !curl -fLo ~/.vim/autoload/plug.vim --create-dirs
                \ https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
    autocmd VimEnter * PlugInstall --sync | source $MYVIMRC
endif

" Specify a directory for plugins
call plug#begin('~/.vim/plugged')

Plug 'jiangmiao/auto-pairs'
Plug 'junegunn/fzf', { 'dir': '~/.fzf', 'do': './install --all' }
Plug 'junegunn/fzf.vim'
Plug 'Chiel92/vim-autoformat'
Plug 'jacoborus/tender.vim'
Plug 'scrooloose/nerdtree', {'on': 'NERDTreeToggle'}
Plug 'itchyny/lightline.vim'
Plug 'kshenoy/vim-signature'
Plug 'terryma/vim-multiple-cursors'
Plug 'nvie/vim-flake8'
Plug 'tpope/vim-fugitive'
Plug 'vim-scripts/taglist.vim'
Plug 'vim-syntastic/syntastic'
Plug 'Valloric/YouCompleteMe'

" Initialize plugin system
call plug#end()

" Shotkeys
" <F7> flake8 using syntastoc
map ; :Files<CR>
nmap <F5> :NERDTreeToggle<CR>
nnoremap tt :tabnext<CR>
nnoremap <silent> <F9> :TlistToggle<CR>
noremap <buffer> <F8> :Autoformat<CR>

" Vim colorscheme
if exists('+termguicolors')
  let &t_8f = "\<Esc>[38;2;%lu;%lu;%lum"
  let &t_8b = "\<Esc>[48;2;%lu;%lu;%lum"
  set termguicolors
endif
colorscheme tender

" YCMD config
set encoding=utf-8

" taglist configs
let Tlist_Use_Right_Window=1

" Lightline config
set laststatus=2
if !has('gui_running')
    set t_Co=256
endif
let g:lightline = {
            \ 'colorscheme': 'tender',
            \ 'active': {
            \    'left': [['mode', 'paste'],
            \             ['gitbranch', 'readonly', 'filename', 'modified']]
            \ },
            \ 'component_function': {
            \   'gitbranch': 'fugitive#head'
            \ },
            \ }
