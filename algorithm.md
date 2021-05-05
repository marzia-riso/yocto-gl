# Immagini
1. Modello con poligoni sopra e intersezioni visualizzate.
2. Modello triangolato lungo i bordi dei poligoni (si vedono i mesh-edges)
3. Celle colorate di colori diversi (flood-filll).
4. Aggiungiamo labeling in sovraimpressione alle celle come testo. Celle con lo stesso labeling diventano dello stesso colore.
5. Mostriamo risultato dell’operazione booleana, disegnando i control points dei nuovi poligoni risultanti.

# 1 [slice_mesh]  
- Creiamo hashgrid:
  - Spezziamo i poligoni originali in polilinee che attraversano i triangoli. Una polilinea in un triangolo può essere aperta oppure chiusa
  - Ogni punto dei poligoni processati viene aggiunto alla mesh
    - In caso collassiamo i punti con i vertici dei triangoli o con altri punti già inseriti nello stesso triangolo
  - (Immagine piccolina per spiegare cosa sono le polilinee)
  
- Calcoliamo i punti di intersezione tra polilinee in uno stesso triangolo. 
  - I punti di intersezione sono aggiunti nell'hashgrid alle polilinee prese in considerazione e quindi anche nella mesh.
  
- I punti inseriti a mano e i punti di intersezione sono salvati come punti di controllo
  - Per i punti di intersezione calcoliamo anche i poligoni che li hanno generati (servono successivamente in fase di ricostruzione del bordo)
  
# 2 [slice_mesh]
- Per ogni faccia dell'hashgrid
  - Calcoliamo i nodi di input per CDT
  - Calcoliamo i vincoli di edge per CDT
  - Triangoliamo con CDT che restituisce triangoli e le loro adianceze (locali)
  - Usiamo i triangoli locali per aggiornare i triangoli globali 
  - Usiamo le adiacenze locali per aggiornare le adiacenze globali
  - Aggiustiamo le adiancenze dei triangoli adiancenti a quelli triangolati [update_face_adjacencies]
  - Calcoliamo border-tags
    - Creiamo hashmap che mappa gli edge che embeddano i poligoni ad un insieme di poligoni
    - Se sono presenti insiemi di poligoni, associamo ad ognuno di loro un indice > numero di poligoni originali
    - Per ogni triangolo triangolato e per ogni edge di quel triangolo
      - Se l'edge sta nell'hashmap, tagghiamo quel lato del triangolo come interno al poligono o ad un insieme (ovvero tramite l'indice che è il valore associato all'edge nell'hashmap)
      - Se l'edge al contrario sta nell'hashmap, tagghiamo quel lato del triangolo come esterno al poligono o all'insieme (ovvero tramite l'indice che è il valore associato all'edge nell'hashmap)  
  

# 3 [flood-fill]
- Il flood-fill trova le celle e genera il grafo delle adiacenze tra esse. In questo grafo c'è un nodo per ogni cella. Ogni cella è connessa a tutte le celle a cui è adiacente sulla superficie.
L’arco è etichettato con le seguenti informazioni:
- Poligono che viene attraversato andando dalla cella a quella adiacente °(o indice di un set di poligoni)
- Un bit che rappresenta l'orientamento del/dei poligono/i attraversato/i (ovvero se sto entrando o uscendo da esso)
    
- Inizializziamo lo stack con l'ultima faccia della mesh
- Finchè lo stack è pieno:
  - Estaiamo l'ultima faccia dallo stack
  - Se la faccia è già stata taggata come appartenente ad una cella, allora skippiamo
  - Aggiungiamo una nuova cella al risultato
  - Ci espandiamo tramite il flood-fill fino a che non troviamo un bordo taggato (che rappresenta una possibile nuova cella)

    neighbor_cell = tag della faccia adiacente all'edge corrente
    if (neighbor_cell >= 0) //Se la cella è stata già visitata
        if (neighbor_cell == cell_id)
            if (crossing border)
                connetti cella a se stessa tramite l'arco di valore +p
            else
                continue //ho già visitato durante il flood-fill di questa cella
        else
            colleghiamo cella a neighbor tramite il poligono o l'insieme di poligoni di indice p
            (se è un insieme di poligoni mettiamo solamente l'arco positivo)

    else //se non ho mai visitato la faccia adiacente
        if (crossing border)
            aggiungo neighbor al cell_stack //mi espanderò su questa faccia dopo aver completato il flood-fill di questa cella
        else
            aggiungo neighbor al face stack //mi espanderò su questa faccia

Una volta trovato il grafo di adiacenza delle celle eseguiamo un passo di post processing per gestire i casi in cui ci siano più poligoni passanti per lo stesso edge
(update_virtual_adjacencies)
  - scorriamo sull'adiancenza tra celle 
  - se un arco da cell a neighbor è taggato da un indice > numero di poligoni allora aggiungiamo una cella virtuale vuota che modella questi collegamenti
  - Estraiamo i poligoni coinvolti tramite l'indice e per ogni poligono creiamo un collegamento tra la cella virtuale e cell oppure neighbor 
    - Gli indici positivi indicano i collegamenti tra la cella virtuale e cell, mentre gli indici negativi sono per i collegamenti tra la cella virtuale e neighbor

# 4 [compute_cell_labels] and [propagate_cell_labels]
- Calcoliamo i cicli presenti nel grafo delle adiacenze tra celle.
  - Per ogni arco coinvolto in un ciclo tagghiamo la cella destinazione dell'arco come interna al poligono identificato dall'arco. 

- Calcoliamo le celle ambiente da cui partire per propagare le etichette
  - Una cella ambiente è una cella senza archi entranti nel grafo delle adiacenze 
  - Se sono presenti più celle ambiente calcoliamo quali sono quelle che nel grafo delle adiacenze hanno distanza massima dalle foglie (ovvero sono esterne a tutti i poligoni)
    - I cicli vengono trattati come super-nodi
  
- Trovate le celle ambiente propaghiamo i label nel grafo delle adiacenze.
  - La propagazione è analoga a mesh arrangements, ma gli archi negativi sono seguiti solamente se il nodo vicino non è mai stato visitato
  
  - Calcoliamo il nuovo label del nodo vicino considerando il poligono di collegamento
    - Se sto attraversando un edge di un ciclo non effettuo la propagazione sulle componenti della label relative agli archi del ciclo 
    - Se il vicino non è mai stato visitato allora il label è esattamente uguale a quello del nodo di partenza incrementato. 
    - Se ho già visitato il nodo tramite altri percorsi allora prendo il valore massimo componente per componente.
    - Se ho effettuato almeno un aggiornamento e non ho già quel nodo nello stack allora lo aggiungo
  
- Nel caso in cui la label sia > 1 applichiamo la even-odd rule
  - Nelle self-intersections posso entrare in un poligono più volte senza esserne prima uscito)

# 5 [compute_bool_operation] and [compute_shape_borders]
- Le operazioni booleane tra due poligoni vengono effettuate con una selezione delle celle che li compongono in base ai labelling. 
- I poligoni coinvolti nell'operazione sono chiamati generatori e il risultato dell'operazione booleana è una nuova shape (insieme di celle)
  
- Alla fine ricostuiamo il bordo delle shape, rappresentandolo come se fosse un poligono iniziale
  - Calcoliamo ricorsivamente tutti i poligoni generatori che hanno prodotto la shape
  - Calcoliamo le componenti connesse delle celle che formano la shape
  - Calcoliamo il bordo di ogni componente connessa
    - Estraiamo la sequenza (non ordinata) degli archi della mesh che compongono il bordo
    - Riordiniamo la sequenza ed estraiamo i punti di controllo finali del bordo
      - tra i punti di controllo salviamo anche dei punti di intersezione solo se si sono formati dall'intersezione di due poligoni entrambi coinvolti nei generatori della shape         



