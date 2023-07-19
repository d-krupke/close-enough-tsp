# Instances of the Carrabs2020 benchmark

Read a dictionary with the instances via:
    
    ```python
    import gzip
    import json

    with open("benchmark.json.gz", "rb") as f:
    instances = json.loads(gzip.decompress(f.read()).decode("utf-8"))
    ```

It has the shape:

    ```json
    {'bonus1000rdmRad': {
        'disks': [
            {'x': 8.842438164,'y': 89.89183088,'r': 9.248878075},
            {'x': 32.77989789, 'y': 83.25231939, 'r': 4.247864371},
            ...
        ],
        'carrabs2020_upper_bound': 955.0059236404267,
        'origin': 'https://github.com/CerroneCarmine/CETSP/tree/master'},
        },
    ...
    }
    ```

The instances with variable overlap ratios are duplicated, such that the radii
of the disks don't have to be scaled. You can identify these instances by
a suffix `(0.XX)` with the overlap ratio.

> Caveat: The depots in the original files were marked as comments. In these files they appear as the first disk
> with a "meta": "depot" attribute.