import cv2 as cv #librería de ope cv
import numpy as np #el as da un como pseudónimo

if __name__== "__main__":

    video_path = "/dev/video0"   #Este puerto se cnoecta a la cámara para capturar cada 
    cap = cv.VideoCapture(video_path) #ve y abre ese video
    Area_min = 200   #150
    Area_max = 20000 #2000
    

    if not cap.isOpened(): #Por si no puede abrir el video
        print("Cannot open video")
        exit()

    while True: 
        ret, originalFr = cap.read() #lee cada uno de los fotográmmas, image = frame original

        if not ret:
            print("Can't recive image")
            break   #si no hay frame se sale del while

        lineas_validas = []

        #ROI
        h, w = originalFr.shape[:2]
        roi = originalFr[int(h*0.6):h, :] #60% de altura, : --> todas las columnas

        roi_h, roi_w = roi.shape[:2]

        #Paraponer el corte vertical
        #x1 = int(w * 0.61)
        #x2 = int(w * 0.97)

        #roi = originalFr[:, x1:x2]

        #convertir de BGR a gris
        gris_image = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)

        #Suavizamos y usamos gaussiano para reducir el ruido antes de analizar la imagen
        blurred = cv.GaussianBlur(gris_image, (5, 5), 0)

        #Threshold the image to binary, tenemos que ir probando y cambiar ese 80 pq por perturbaciones el negro no será 100% negro
        #80 como umbral
        _, binary = cv.threshold(blurred,80, 255, cv.THRESH_BINARY_INV) 

        #Mascara trapezoidal, hacemos esto para quedarnos en una zona parecida a un carril
        top_width = int(roi_w * 0.6)
        top_y = int(roi_h * 0.3)
        trapezoid = np.array([[
                    ((roi_w - top_width) // 2, top_y),
                    ((roi_w + top_width) // 2, top_y),
                    (roi_w, roi_h),
                    (0, roi_h)
                    ]], dtype=np.int32)
        
        mask = np.zeros((roi_h, roi_w), dtype=np.uint8)
        cv.fillPoly(mask, trapezoid, 255)
        binary_masked = cv.bitwise_and(binary, binary, mask=mask)

        #Morphological operations
        kernel = np.ones((3,3), np.uint8)
        morph = cv.erode(binary_masked, kernel, iterations=1)
        morph = cv.dilate(morph, kernel, iterations=1)
        
        #El problema de los momentos que solo detecta una cosa pero con este algoritmo podemos hacer que si funcione
        #num_labels: regiones encontrada, labels: etiquetas, stats: datos de cada región

        num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(morph, connectivity=8)

        #sirve para separar componentes conectados, es el algoritmo exxtra que ayuda a momentos
        # = cv.connectedComponentsWithStats(binary)----checar

        #centroide
        #num_labels, labels, stats, centroids = cv2,connectedComponentsWithStats(src[, connectivity[,ltype]])

        zona_central_min = int(roi_w * 0.40)
        zona_central_max = int(roi_w * 0.60)

        output_final = roi.copy()

        # Dibujar trapecio
        cv.polylines(output_final, trapezoid, True, (255, 0, 255), 2)

        # Dibujar zona central
        cv.line(output_final, (zona_central_min, 0), (zona_central_min, roi_h), (255, 0, 0), 2)
        cv.line(output_final, (zona_central_max, 0), (zona_central_max, roi_h), (255, 0, 0), 2)
        

        # Referencia central
        ref_x = roi_w // 2

        # Dibujar línea central de referencia
        cv.line(output_final, (ref_x, 0), (ref_x, roi_h), (255, 0, 0), 2)

        candidatos = []

        for i in range(1, num_labels):
            x, y, bw, bh, area = stats[i]
            cx, cy = centroids[i]

            if Area_min <= area <= Area_max:
                candidatos.append((cx, cy, area, x, y, bw, bh))

        if len(candidatos) > 0:
        # Elegir solo una línea: la más cercana al centro
            cx, cy, area, x, y, bw, bh = min(
            candidatos,
            key=lambda linea: abs(linea[0] - ref_x)
            )

            lineas_validas.append((cx, cy, area, x, y, bw, bh))

            cv.rectangle(output_final, (x, y), (x + bw, y + bh), (255, 0, 0), 2)
            cv.circle(output_final, (int(cx), int(cy)), 4, (0, 0, 255), -1)
            cv.putText(output_final, f"({int(cx)},{int(cy)})",
                        (int(cx) + 5, int(cy)),
                        cv.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (0, 255, 255),
                          1)

            # Error respecto al centro
            error_x = int(cx - ref_x)

            cv.line(output_final, (ref_x, roi_h), (int(cx), int(cy)), (0, 255, 255), 2)

            cv.putText(output_final, f"error: {error_x}",
                    (10, 25),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2)
        else:
            cv.putText(output_final, "Linea no detectada",
                    (10, 25),
                    cv.FONT_HERSHEY_SIMPLEX,
                     0.7,
                         (0, 0, 255),
                     2)   

                
            
        

        #Muestra los resultados
        cv.imshow('frame original', originalFr)
        #cv.imshow('binary', binary)
        #cv.imshow('blurred', blurred)
        cv.imshow('morph', morph)
        cv.imshow('componentes', output_final)
        cv.imshow('binary_masked', binary_masked)

        #cv.imshow('clean', clean)

        k = cv.waitKey(25) & 0xFF
        if k == 27:
            break
  

    cap.release() #libera la memoría del video
    cv.destroyAllWindows() #cierra ventanas
