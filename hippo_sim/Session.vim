let SessionLoad = 1
let s:so_save = &g:so | let s:siso_save = &g:siso | setg so=0 siso=0 | setl so=-1 siso=-1
let v:this_session=expand("<sfile>:p")
silent only
silent tabonly
cd ~/fav/ros2_underlay/src/hippo_simulation/hippo_sim
if expand('%') == '' && !&modified && line('$') <= 1 && getline(1) == ''
  let s:wipebuf = bufnr('%')
endif
let s:shortmess_save = &shortmess
if &shortmess =~ 'A'
  set shortmess=aoOA
else
  set shortmess=aoO
endif
badd +10 models/bluerov/urdf/bluerov.xacro
badd +9 models/bluerov/urdf/bluerov_macro.xacro
badd +10 /opt/ros/iron/share/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro
badd +1 ~/alpha_arm/alpha_sim/models/alpha/urdf/alpha_5_macro.urdf.xacro
badd +1 CMakeLists.txt
badd +1 /opt/ros/iron/share/robotiq_description/urdf/robotiq_gripper.ros2_control.xacro
argglobal
%argdel
$argadd models/bluerov/urdf/bluerov.xacro
set stal=2
tabnew +setlocal\ bufhidden=wipe
tabnew +setlocal\ bufhidden=wipe
tabnew +setlocal\ bufhidden=wipe
tabrewind
edit models/bluerov/urdf/bluerov.xacro
argglobal
balt models/bluerov/urdf/bluerov_macro.xacro
setlocal fdm=expr
setlocal fde=nvim_treesitter#foldexpr()
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=6
setlocal nofen
let s:l = 10 - ((9 * winheight(0) + 28) / 57)
if s:l < 1 | let s:l = 1 | endif
keepjumps exe s:l
normal! zt
keepjumps 10
normal! 090|
tabnext
edit CMakeLists.txt
argglobal
balt models/bluerov/urdf/bluerov.xacro
setlocal fdm=expr
setlocal fde=nvim_treesitter#foldexpr()
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=6
setlocal fen
let s:l = 101 - ((36 * winheight(0) + 28) / 57)
if s:l < 1 | let s:l = 1 | endif
keepjumps exe s:l
normal! zt
keepjumps 101
normal! 0
tabnext
edit /opt/ros/iron/share/robotiq_description/urdf/robotiq_gripper.ros2_control.xacro
argglobal
balt CMakeLists.txt
setlocal fdm=expr
setlocal fde=nvim_treesitter#foldexpr()
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=6
setlocal nofen
let s:l = 4 - ((3 * winheight(0) + 28) / 57)
if s:l < 1 | let s:l = 1 | endif
keepjumps exe s:l
normal! zt
keepjumps 4
normal! 024|
tabnext
edit ~/alpha_arm/alpha_sim/models/alpha/urdf/alpha_5_macro.urdf.xacro
argglobal
balt models/bluerov/urdf/bluerov_macro.xacro
setlocal fdm=expr
setlocal fde=nvim_treesitter#foldexpr()
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=6
setlocal nofen
let s:l = 35 - ((6 * winheight(0) + 28) / 57)
if s:l < 1 | let s:l = 1 | endif
keepjumps exe s:l
normal! zt
keepjumps 35
normal! 0
tabnext 1
set stal=1
if exists('s:wipebuf') && len(win_findbuf(s:wipebuf)) == 0 && getbufvar(s:wipebuf, '&buftype') isnot# 'terminal'
  silent exe 'bwipe ' . s:wipebuf
endif
unlet! s:wipebuf
set winheight=1 winwidth=20
let &shortmess = s:shortmess_save
let s:sx = expand("<sfile>:p:r")."x.vim"
if filereadable(s:sx)
  exe "source " . fnameescape(s:sx)
endif
let &g:so = s:so_save | let &g:siso = s:siso_save
set hlsearch
nohlsearch
let g:this_session = v:this_session
let g:this_obsession = v:this_session
doautoall SessionLoadPost
unlet SessionLoad
" vim: set ft=vim :
