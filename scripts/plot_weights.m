function weights = plot_weights(filename, n_neurons)

	x= 1:n_neurons;
	y=x;
	weights = load(filename);
	surf(x,y,weights);

end
