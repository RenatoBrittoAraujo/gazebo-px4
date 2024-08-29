estado = "DECOLAR"
mapa_transicao_estados = {
    "DECOLAR": "SOBE_VOO",
    "SOBE_VOO": "ESTA_VOADO_ESTAVEL",
}


class Controle:

    def decola(self):
        self.ros.takeoff()

    def sobe_voo(self, x, y):
        self.ros.moveTo(x, y, 3.0)

    def esta_voando_estavel(self) -> bool:
        pass

    def main(self):
        while True:
            chamar_função_de_estado(estado)

            estado_novo = mapa_transicao_estados[estado]
            estado = estado_novo
