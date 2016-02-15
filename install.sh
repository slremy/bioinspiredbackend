sudo apt-get update
sudo apt-get -y install python-webpy screen nginx
wget https://raw.githubusercontent.com/slremy/bioinspiredbackend/master/nginx.conf
wget https://raw.githubusercontent.com/slremy/bioinspiredbackend/master/websingleton.py
wget https://raw.githubusercontent.com/slremy/bioinspiredbackend/master/run
sudo mv nginx.conf /etc/nginx/nginx.conf
sudo service nginx restart 
