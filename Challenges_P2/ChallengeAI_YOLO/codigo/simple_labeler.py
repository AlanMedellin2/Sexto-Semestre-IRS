import cv2
from pathlib import Path




IMG_DIR = Path("real_extra/images/all")
LBL_DIR = Path("real_extra/labels/all")
LBL_DIR.mkdir(parents=True, exist_ok=True)

CLASSES = ["stop", "straight", "turn_right", "turn_left", "speed_limit_30"]
CLASS_TO_ID = {name: i for i, name in enumerate(CLASSES)}



IMAGE_EXTS = {".jpg", ".jpeg", ".png"}

images = sorted([p for p in IMG_DIR.iterdir() if p.suffix.lower() in IMAGE_EXTS])

current_box = []
boxes = []
current_img = None
current_img_path = None
display_img = None




def save_yolo_labels(img_path, img_shape, boxes_list):
    h, w = img_shape[:2]
    label_path = LBL_DIR / f"{img_path.stem}.txt"

    with open(label_path, "w") as f:
        for cls_id, x1, y1, x2, y2 in boxes_list:
            x_min = min(x1, x2)
            y_min = min(y1, y2)
            x_max = max(x1, x2)
            y_max = max(y1, y2)

            xc = ((x_min + x_max) / 2) / w
            yc = ((y_min + y_max) / 2) / h
            bw = (x_max - x_min) / w
            bh = (y_max - y_min) / h

            f.write(f"{cls_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}\n")


def redraw():
    global display_img
    display_img = current_img.copy()





    for cls_id, x1, y1, x2, y2 in boxes:
        cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = CLASSES[cls_id]
        cv2.putText(display_img, label, (x1, max(20, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    if len(current_box) == 2:
        x1, y1 = current_box[0]
        x2, y2 = current_box[1]
        cv2.rectangle(display_img, (x1, y1), (x2, y2), (255, 0, 0), 2)

    help_text = "n: siguiente | b: anterior | u: deshacer | s: guardar | q: salir"
    cv2.putText(display_img, help_text, (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)





def mouse_callback(event, x, y, flags, param):
    global current_box, boxes

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(current_box) == 0:
            current_box = [(x, y)]
        elif len(current_box) == 1:
            current_box.append((x, y))

            print("\nSelecciona clase:")
            for i, name in enumerate(CLASSES):
                print(f"  {i}: {name}")

            cls_input = input("Clase (numero): ").strip()

            if cls_input.isdigit() and int(cls_input) in range(len(CLASSES)):
                cls_id = int(cls_input)
                (x1, y1), (x2, y2) = current_box
                boxes.append((cls_id, x1, y1, x2, y2))
                print(f"Caja agregada: {CLASSES[cls_id]}")
            else:
                print("Clase invalida, caja descartada.")

            current_box = []

        redraw()





def load_existing_labels(img_path, img_shape):
    existing = []
    label_path = LBL_DIR / f"{img_path.stem}.txt"
    if not label_path.exists():
        return existing

    h, w = img_shape[:2]
    lines = label_path.read_text().strip().splitlines()

    for line in lines:
        parts = line.strip().split()
        if len(parts) != 5:
            continue

        cls_id = int(float(parts[0]))
        xc = float(parts[1]) * w
        yc = float(parts[2]) * h
        bw = float(parts[3]) * w
        bh = float(parts[4]) * h

        x1 = int(xc - bw / 2)
        y1 = int(yc - bh / 2)
        x2 = int(xc + bw / 2)
        y2 = int(yc + bh / 2)

        existing.append((cls_id, x1, y1, x2, y2))

    return existing




idx = 0
cv2.namedWindow("Labeler", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Labeler", mouse_callback)



while 0 <= idx < len(images):
    current_img_path = images[idx]
    current_img = cv2.imread(str(current_img_path))
    if current_img is None:
        print(f"No se pudo abrir {current_img_path}")
        idx += 1
        continue

    boxes = load_existing_labels(current_img_path, current_img.shape)
    current_box = []
    redraw()

    print(f"\nImagen {idx+1}/{len(images)}: {current_img_path.name}")
    print("Haz 2 clics para dibujar caja. Si no hay señal, guarda vacío y pasa.")



    while True:
        cv2.imshow("Labeler", display_img)
        key = cv2.waitKey(20) & 0xFF

        if key == ord("s"):
            save_yolo_labels(current_img_path, current_img.shape, boxes)
            print(f"Guardado: {current_img_path.stem}.txt")
            
        elif key == ord("u"):
            if boxes:
                removed = boxes.pop()
                print(f"Deshecha caja: {CLASSES[removed[0]]}")
                redraw()
                
        elif key == ord("n"):
            save_yolo_labels(current_img_path, current_img.shape, boxes)
            idx += 1
            break
            
        elif key == ord("b"):
            save_yolo_labels(current_img_path, current_img.shape, boxes)
            idx = max(0, idx - 1)
            break
            
        elif key == ord("q"):
            save_yolo_labels(current_img_path, current_img.shape, boxes)
            cv2.destroyAllWindows()
            print("Saliendo...")
            raise SystemExit
            
            

cv2.destroyAllWindows()
print("Terminado.")
