# Download and setup ade
[ ! -d  ~/.local/bin ] && mkdir -p ~/.local/bin

echo "export PATH=$HOME/.local/bin\${PATH:+:\${PATH}}" >> $HOME/.bashrc
source $HOME/.bashrc

cd ~/.local/bin && wget https://gitlab.com/ApexAI/ade-cli/uploads/591bf9c7ef766cf859749b21afa700b7/ade+x86_64
mv ~/.local/bin/ade+x86_64 ~/.local/bin/ade
chmod +x ~/.local/bin/ade

ade update-cli
ade --version

# Setup GitHub Container Registry credentials
export GH_TOKEN=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
echo $GH_TOKEN | docker login ghcr.io -u AndreaFinazzi --password-stdin