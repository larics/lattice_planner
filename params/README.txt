	
Datoteka "params.mat" generirana je u MATLAB-u i sadrži sljedeće parametre:
	
"vars"	->	matrica čiji retci povezuju početnu orijentaciju, koordinate ciljnog čvora, konačnu orijentaciju te cijenu pojedinog segmenta u latici
"cost"	->	polje cell-ova koji sadrže koordinate svih čelija kroz koje vozilo prolazi prilikom gibanja pojedinim segmentom putanje
"edges"	->	matrica cell-ova koji sadrže sljedeće informacije o svakom segmentu u latici: dx, dy, fif, x, y, fi, sf, inTime, outTime, R, cost.
"stop"	->	polje cell-ova koji sadrže koordinate prostornih čelija koje vozilo okupira u mirovanju pri pojedinoj orijentaciji
