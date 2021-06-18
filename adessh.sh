if ! [ -f ~/.ssh/id_rsa.pub ]; then
  ssh-keygen -b 2048 -t rsa -f ~/.ssh/id_rsa -q -N ""
fi
echo $USER | ssh-copy-id $USER@localhost -p2222
ssh-keygen -f "$HOME/.ssh/known_hosts" -R "[localhost]:2222"
