wget http://192.168.3.5:5081/dav/rebeater/CICD/testdata/demo_A15.tar.gz
wget http://192.168.3.5:5081/dav/rebeater/CICD/testdata/demo_a15.yml
tar -xvf demo_A15.tar.gz
./App/bin/PostDataFusion demo_a15.yml
cd Tools && python ErrorAnalyse.py ../demo_a15.yml 0.04 0.05