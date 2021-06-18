if ! [ -f ~/.ssh/id_rsa.pub ]; then
  ssh-keygen -b 2048 -t rsa -f ~/.ssh/id_rsa -q -N ""
fi
echo $USER | ssh-copy-id $USER@localhost -p2222
ssh-keygen -f "$HOME/.ssh/known_hosts" -R "[localhost]:2222"

export CR_PAT=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
echo $CR_PAT | docker login ghcr.io -u AndreaFinazzi --password-stdin