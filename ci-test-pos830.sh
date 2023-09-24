wget http://192.168.3.5:5081/dav/rebeater/CICD/testdata/demo_pos830.tar.gz
wget http://192.168.3.5:5081/dav/rebeater/CICD/testdata/demo_pos830.yml
tar -xvf demo_pos830.tar.gz
./App/bin/PostDataFusion demo_pos830.yml
python Tools/ErrorAnalyse.py demo_pos830.yml 0.04 0.05