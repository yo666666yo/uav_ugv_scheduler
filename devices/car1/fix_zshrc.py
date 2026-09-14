import os, re, shutil
p = os.path.expanduser('~/.zshrc')
with open(p) as f:
    txt = f.read()
shutil.copy(p, p + '.bak')
txt = re.sub(r'# ========== zrdds ==========.*?# ===========================\n', '', txt, flags=re.S)
new_block = '''
# ========== zrdds (all-car) ==========
export ZRDDS_HOME=/usr/ZRDDS/ZRDDS-2.4.4
export LD_LIBRARY_PATH=/usr/ZRDDS/ZRDDS-2.4.4/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_zrdds_dynamic_cpp
export RMW_ZRDDS_XML_PATH=/home/ubuntu/snowman/ZRDDS/ZRDDS-2.4.4/ZRDDS_QOS_PROFILES.xml
# ==================================
'''
if 'all-car' not in txt:
    txt = txt.rstrip() + '\n' + new_block
with open(p, 'w') as f:
    f.write(txt)
print('ZSHRC_OK')
print('--- tail ---')
print(txt[-400:])
