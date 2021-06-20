# Download and setup ade
cd ~/.local/bin && wget https://gitlab.com/ApexAI/ade-cli/uploads/591bf9c7ef766cf859749b21afa700b7/ade+x86_64
mv ~/.local/bin/ade+x86_64 ~/.local/bin/ade
chmod +x ~/.local/bin/ade
ade update-cli
ade --version

# Setup GitHub Container Registry credentials
export CR_PAT=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
echo $CR_PAT | docker login ghcr.io -u AndreaFinazzi --password-stdin