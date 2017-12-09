Vagrant.configure(2) do |config|
  config.vm.box = "debian/stretch64"

  config.vm.box_check_update = false
  config.vm.network "public_network"

  config.vm.provider :virtualbox do |vb|
    vb.customize ["modifyvm", :id, "--usb", "on"]
    vb.customize ["usbfilter", "add", "0", "--target", :id,
                  "--name", "XAP-EA-001",
                  "--vendorid", "0x10c4", "--productid", "0x8BDE"]
  end

  config.vm.synced_folder ".", "/git"

  config.vm.provision "shell", inline: <<-SHELL
    #BACKPORTS=/etc/apt/sources.list.d/jessie-backports.list
    #if [ ! -e "$BACKPORTS" ]; then
    #   echo "deb http://ftp.debian.org/debian jessie-backports main" > $BACKPORTS
    #fi

    sudo apt-get update
    sudo apt-get install -y gcc make                          \
                            dpkg-dev usbutils
    sudo apt-get install -y linux-headers-$(uname -r)
    #sudo apt-get -t jessie-backports                          \
    #             install -y linux-image-4.9.0-0.bpo.3-amd64   \
    #                        linux-headers-4.9.0-0.bpo.3-amd64
    sudo apt-get autoremove -y

  SHELL
  
end
