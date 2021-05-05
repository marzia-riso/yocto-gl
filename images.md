# Paper Images
- Teaser formato da 3 immagini di cui:
  - Modello complesso con caso euclideo
  - Non euclideo con cosa che si attorciglia (sia davanti che dietro)

[x] Sequenza di immagini che spiega l'algoritmo (vedi anche algorithm.md)
  - Modello con i poligoni iniziali e con intersezioni disegnate
  - Triangolazione (visualizzare bene gli edge embeddati)
  - Identificazione delle celle con colori differenti
  - Celle con la stessa label sono colorate allo stesso modo e con i label in sovraimpressione
  - Risultato finale della booleana con annessi control-point e brodi 
  
- Triangolazione:
  - Normale
  - Hashgrid + intersezioni
  - Triangolazione
  - Border faces dentro

[x] Spiegazione della triangolazione e della creazione dei vincoli
  - Immagine in cui facciamo vedere cosa accade all'intendo di un triangolo (+ i suoi vicini?), rappresentando quali sono gli edge aggiunti alla triangolazione con CDT

[x] Spiegazione del flood-fill
  - Sequenza di immagini che fanno vedere come si espande il flood-fill (e in contemporanea come si crea il grafo delle adiacenze)

- Calcolo delle celle ambiente fatto bene (?)
  - Esempio su banane chiuse, spiegando perché le celle iniziali vengono poi scartate (?)

- Esempi di propagazione delle labels
  - Caso normale (forse poco interessante) ABC.json 
  - Multiple paths (sul toro)
  - Cicli (sul toro) -> torus-cycles + fix(?)
  - Self loop 

[x] Labelling con tante self-intersections
  - Far vedere l'otto sia al dritto che al rovescio 
  - Grafo uguale

- Risultati, prendere modelli belli da thingi10k o da altri paper (Crane!) e disegnare a mano diversi esempi tipo
  - Tante intersezioni tra poligoni
  - Numero alto di suddivisioni delle curve
  - Poligoni che passano su tanti triangoli
  - Poligoni che passano su un triangolo solo

  - Idee:
    - Mappa svg con intersezioni 
    - Caso non euclideo
    - Caso euclideo con esempio complesso
    - 5 grafi ovvi con grafi fighi
    - Texture bombing

- Test su Thingi10k (da mostrare a parte nei supplemental - 4321 mesh totali)
  - Qui va capito se mostrare il risultato di una booleana come area colorata o solamente tramite il bordo 


