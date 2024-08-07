import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yuct/work/aichallenge-2024/aichallenge/workspace/src/aichallenge_submit/object_subscriber/install/object_subscriber'
