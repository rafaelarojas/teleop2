# TurtleBot Teleoperado II
Incrementar o sistema do turtlebot teleoperado para incluir conceitos de streaming de imagens.

![demonstração](image.gif)

Para ver o vídeo completo de demonstração, [clique aqui](https://drive.google.com/file/d/10XEeT9QCCaneT5DlxN_ckrrdG_zL5hOW/view?usp=sharing)

---

### Instalação

Certifique-se de ter o ROS2 instalado em seu sistema. Além disso, certifique-se de ter todas as dependências instaladas, elas estão disponíveis no arquivo `requirements.txt` na raíz do projeto, para isso utilize o comando:

```bash
pip install -r requirements.txt
```

Clone este repositório em seu ambiente:


```bash
git clone https://github.com/rafaelarojas/teleop2
```

Além disso, clone este repositório também no Robô TurtleBot, para executar o Bringup. Para clonar no robô, utiliza-se o mesmo comando descrito acima.

---

### Execução

Para executar, certifique-se de estar conectado com o robô, e no seu computador estar na pasta `workspace/src/visor_pkg/visor_pkg`. Use o comando:

```bash
streamlit run main.py
```

Com isso a interface será aberta no seu navegador de preferência.