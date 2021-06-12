
config_file_name="pointpillars/hv_pointpillars_secfpn.py"

if [ $# -gt 0 ]; then
  echo "use external arguments..."
  config_file_name=$1
fi

# https://blog.csdn.net/seulzz/article/details/86512843
# https://www.w3cschool.cn/bashshell/bashshell-h19y37lm.html

work_dir_name=${config_file_name##*/}
work_dir_name=${work_dir_name%.*}

echo ${config_file_name}
echo ${work_dir_name}  # hv_pointpillars_secfpn