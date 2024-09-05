make_nocam:
	./run_nocam.sh

install:
	sudo -v

	# execute all scripts in folder passos
	cd passos
	for file in ./*.sh; do
		bash $file
	done