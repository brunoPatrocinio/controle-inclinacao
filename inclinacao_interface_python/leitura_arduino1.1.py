from tkinter import *
from tkinter import messagebox
from tkinter import ttk
import serial
import cv2 as cv
import numpy as np

#Lê a imagem do arquivo
src = cv.imread('d:/disp.png')

srcTri = np.array( [[0, 0], [src.shape[1] - 1, 0], [0, src.shape[0] - 1]] ).astype(np.float32)
dstTri = np.array( [[0, src.shape[1]*0.33], [src.shape[1]*0.85, src.shape[0]*0.25], [src.shape[1]*0.15, src.shape[0]*0.7]] ).astype(np.float32)
warp_mat = cv.getAffineTransform(srcTri, dstTri)
warp_dst = cv.warpAffine(src, warp_mat, (src.shape[1], src.shape[0]))
#Rotaciona a imagem após empena-la levemente
center = (warp_dst.shape[1]//2, warp_dst.shape[0]//2)
angle = -17
scale = 1
rot_mat = cv.getRotationMatrix2D( center, angle, scale )
warp_rotate_dst = cv.warpAffine(warp_dst, rot_mat, (warp_dst.shape[1], warp_dst.shape[0]))

#Mostra a imagem do objeto na tela
cv.imshow('Warp + Rotate', warp_rotate_dst)

# Config. da Janela da interface
janela = Tk()
janela.title("Inclinação GUI")
janela['bg'] = ('lightgray')
janela.geometry('1024x450')

# Coordenadas
x = 130
y = 150
x2 = 450
y2 = 150

# Cria o Canvas
tela = Canvas(janela, height=400, width=600, bg="gray")
tela.place(x=400, y=10)

# Variaveis de Conexão do arduino
global numPorta
global baudRate

# Funções
def sair():
    #arduino.close()
    janela.destroy()

def desconectar():
    try:
        arduino.close()
        arduino.flush()
    except:
        messagebox.showinfo("Mensagem", "Nada Conectado.")

def reading():
    mensagem = arduino.read_until()
    # dado = int.from_bytes(mensagem, byteorder=sys.byteorder, signed=True)
    stringona = mensagem.decode('utf-8')

    # label1['text'] = stringona
    print(label1.cget('text'))
    label1.config(text='Dado: ' + str(stringona))
    janela.after(20, reading)
    # label1.after(200, reading)
    # tela.update_idletasks()
    # print(stringona)
    valor = [stringona]

    for n in valor:
        if n > "215.80":
            direita()
        elif n < "206.20":
            esquerda()
        elif n > "215.80":
            print("Risco de IA")
        elif n < "206.20":
            print("Risco de IA")
        else:
            neutro()

def esquerda():
    y = -10
    x = 0
    x2 = 0
    y2 = 0
    tela.move(linha, x, y)
    # tela.coords(linha, 130, y - 10, 450, 150)
    rot_mat = cv.getRotationMatrix2D(center, angle-10, scale)
    warp_rotate_dst = cv.warpAffine(warp_dst, rot_mat, (warp_dst.shape[1], warp_dst.shape[0]))
    cv.imshow('Warp + Rotate', warp_rotate_dst)

def direita():
    y = +10
    x = 0
    x2 = 0
    y2 = 0
    tela.move(linha, x, y)
    # tela.coords(linha,130,y+10,450,150)
    rot_mat = cv.getRotationMatrix2D(center, angle+10, scale)
    warp_rotate_dst = cv.warpAffine(warp_dst, rot_mat, (warp_dst.shape[1], warp_dst.shape[0]))
    cv.imshow('Warp + Rotate', warp_rotate_dst)

def neutro():
    rot_mat = cv.getRotationMatrix2D(center, angle, scale)
    warp_rotate_dst = cv.warpAffine(warp_dst, rot_mat, (warp_dst.shape[1], warp_dst.shape[0]))
    cv.imshow('Warp + Rotate', warp_rotate_dst)

def conectar():
    try:
        comboSelecao.current()
        numPorta = comboSelecao.get()
        baudRate = comboSelecao2.get()
        global arduino
        arduino = serial.Serial(numPorta, baudRate, 8)
        reading()
    except:
        messagebox.showinfo("Erro", "Nenhuma porta Selecionada ou não encontrada.")
        print("Nenhuma porta selecionada ou não encontrada.")

# Componentes da interface

# Titulo
labelTitulo = Label(janela, text='Painel de Informações', bg='yellow', padx='50', pady='10', cursor='dot',
                    relief='groove', font=12)
labelTitulo.place(x=20, y=10)

# Representação do Dispositivo de Montagem
linha = tela.create_line(x, y, x2, y2, width=50, fill="blue")

# Sub titulo1
labelTitulo2 = Label(janela, text='Inclinação em X:', bg='lightblue', padx='50', pady='10', cursor='dot',
                     relief='groove', font=12)
labelTitulo2.place(x=20, y=60)

# Label de vis. Inclinação eixo X
label1 = Label(janela, text='Dado', bg='lightblue', padx='20', pady='10', cursor='dot', relief='groove', font=8)
label1.place(x=20, y=115)

#Comobox Seleção de portas.
label2 = Label(janela, text='Porta USB:         ', bg='lightblue', padx='12', pady='10', cursor='dot', relief='groove', font=8)
label2.place(x=20, y=164)

comboSelecao = ttk.Combobox(janela, values=["com0", "com1", "com2", "com3", "com4", "com5", "com6", "com7",
                                            "com8", "com9", "com10", "com11"])
comboSelecao.place(x=20, y=212)

#Label do Combobox Seleção do Baud Rate
label3 = Label(janela, text='Baud Rate:  ', bg='lightblue', padx='12', pady='10', cursor='dot', relief='groove', font=8)
label3.place(x=20, y=240)

#Combobox Seleção de Baud Rate
comboSelecao2 = ttk.Combobox(janela, values=["9600", "19200", "38400", "57600", "74880", "115200"])
comboSelecao2.place(x=20, y=290)

#Botão conectar
botaoSelecao = Button(janela, text='Conectar', bg='blue', fg='yellow', padx='20', pady='10', font=10, command=conectar)
botaoSelecao.place(x=120, y=320)

#Botão desconectar
botaoDesconect = Button(janela, text='Desconectar', bg='blue', fg='yellow', padx='10', pady='10', font=10, command=desconectar)
botaoDesconect.place(x=250, y=320)

# botão sair
botao1 = Button(janela, text='Sair', bg='blue', fg='yellow', padx='30', pady='10', font=10, command=sair)
botao1.place(x=20, y=320)

# janela.after(200, reading)
# janela.update_idletasks()

janela.mainloop()