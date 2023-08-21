from core.environment import IsaacHandler



def main():
    environment = IsaacHandler(1/60, 1/60)
    environment.setup()
    environment.run()
if __name__ == "__main__":
    main()
